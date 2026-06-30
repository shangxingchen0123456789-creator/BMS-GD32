#include "app_tasks.h"

#include "FreeRTOS.h"
#include "task.h"

#include "adc_manager.h"
#include "afe_gd30bm2016.h"
#include "balance_manager.h"
#include "bms_comm_service.h"
#include "bms_state.h"
#include "board_support.h"
#include "charge_manager.h"
#include "fault_log.h"
#include "param_storage.h"
#include "power_control.h"
#include "power_manager.h"
#include "power_path_manager.h"
#include "safety_manager.h"
#include "soc_estimator.h"

/*
 * 充电器 FreeRTOS 任务编排。
 *
 * 高频 PWM 波形由 power_control.c 中的 HRTIMER 硬件直接产生，
 * 这里负责较慢速的系统监督循环：
 * - 采集 AFE 与功率板 ADC 数据；
 * - 更新充电状态机；
 * - 计算均衡位图和 SOC；
 * - 发布一份完整状态快照，供 UART 协议任务打包上报。
 *
 * 任务划分原则：
 * - 安全任务优先级最高，只负责快速采样硬故障并关断 Q4/Q5；
 * - 通信任务优先级次高，UART 收发采用中断缓冲，避免阻塞其它任务；
 * - 控制任务只做采样、状态机和目标更新，不直接处理串口字节流；
 * - 心跳任务独立存在，便于判断系统调度器是否仍在运行。
 */
#define CONTROL_PERIOD_MS                      20U
#define FAST_CONTROL_PERIOD_MS                 1U
#define COMM_PERIOD_MS                         10U
#define SAFETY_PERIOD_MS                       2U
#define HEARTBEAT_PERIOD_MS                    500U

#define APP_TASK_PRIORITY_SAFETY               (configMAX_PRIORITIES - 1U)
#define APP_TASK_PRIORITY_FAST_CONTROL         (configMAX_PRIORITIES - 2U)
#define APP_TASK_PRIORITY_COMM                 (configMAX_PRIORITIES - 3U)
#define APP_TASK_PRIORITY_CONTROL              (configMAX_PRIORITIES - 4U)
#define APP_TASK_PRIORITY_HEARTBEAT            (tskIDLE_PRIORITY + 1U)

static bms_power_sample_t s_latest_power_sample;
static uint8_t s_latest_power_sample_valid;

static void App_Store_Latest_Power_Sample(const bms_power_sample_t *sample)
{
    if(sample == 0) {
        return;
    }

    taskENTER_CRITICAL();
    s_latest_power_sample = *sample;
    s_latest_power_sample_valid = 1U;
    taskEXIT_CRITICAL();
}

static uint8_t App_Load_Latest_Power_Sample(bms_power_sample_t *sample)
{
    uint8_t valid;

    if(sample == 0) {
        return 0U;
    }

    taskENTER_CRITICAL();
    valid = s_latest_power_sample_valid;
    if(valid != 0U) {
        *sample = s_latest_power_sample;
    }
    taskEXIT_CRITICAL();

    return valid;
}

uint8_t App_Tasks_Get_Latest_Power_Sample(bms_power_sample_t *sample)
{
    return App_Load_Latest_Power_Sample(sample);
}

static void App_Collect_And_Publish_Status(uint32_t period_ms)
{
    bms_afe_data_t afe;
    bms_power_sample_t power_sample;
    bms_charge_parameters_t parameters;
    bms_status_t status;
    uint8_t digital_power_active;

    /*
     * 统一采集入口。
     * 控制任务每 20 ms 读取一次真实 BM2016 电芯电压和功率板 ADC。
     * 任何硬件读取失败都由底层置故障位，禁止生成模拟值。
     */
    Afe_Gd30bm2016_Poll(&afe);
    if(App_Load_Latest_Power_Sample(&power_sample) == 0U) {
        Adc_Manager_Sample(&afe, &power_sample);
        App_Store_Latest_Power_Sample(&power_sample);
    }

    /*
     * 状态机负责合并 AFE/ADC/外部故障，并根据故障最高优先级规则关断功率路径。
     * period_ms 来自控制任务周期，用于 SOC 等慢速积分逻辑。
     */
    Charge_Manager_Update(period_ms, &afe, &power_sample, &status);
    digital_power_active = Charge_Manager_Is_Digital_Power_Active();

    Charge_Manager_Get_Parameters(&parameters);
    if(digital_power_active != 0U) {
        status.balanceBitmap = 0U;
        Afe_Gd30bm2016_Set_Balance(0U);
    } else {
        status.balanceBitmap = Balance_Manager_Update(&afe, &parameters, status.chargeState);
        Afe_Gd30bm2016_Set_Balance(status.balanceBitmap);
        Soc_Estimator_Update(period_ms, &afe, status.chargeCurrentMa);
    }

    status.socX10 = Soc_Estimator_Get_X10();
    status.timestampMs = Board_Support_Millis();
    Fault_Log_Service(&status, &power_sample);
    Param_Storage_Service(status.timestampMs);
    Bms_State_Set_Status(&status);

    Board_Support_Fault_Led((status.faultBitmap != 0U) ? 1U : 0U);
}

static void App_Safety_Task(void *argument)
{
    TickType_t last_wake;

    (void)argument;

    last_wake = xTaskGetTickCount();
    for(;;) {
        Safety_Manager_Service();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAFETY_PERIOD_MS));
    }
}

/*
 * 主控制任务，周期 20 ms。
 *
 * 这个周期不等同于 PWM 周期。PWM 是 200 kHz 级别的硬件输出，
 * 本任务只负责较慢的监督控制：
 * 1. 从 BM2016 AFE 获取电芯、电池包和温度数据；
 * 2. 从功率板 ADC 获取输入/输出电压、电流和温度；
 * 3. 调用 Charge_Manager_Update() 根据保护阈值和命令更新充电状态；
 * 4. 根据当前阶段决定是否开启被动均衡；
 * 5. 更新 SOC 和共享状态，供通信任务异步读取。
 *
 * 局部结构体放在任务栈中，避免多个模块共享临时缓冲区。
 */
static void App_Fast_Control_Task(void *argument)
{
    TickType_t last_wake;
    bms_power_sample_t power_sample;
    uint32_t reported_faults;
    uint32_t fast_faults;

    (void)argument;

    reported_faults = 0U;
    last_wake = xTaskGetTickCount();
    for(;;) {
        Adc_Manager_Sample_Fast(&power_sample);
        App_Store_Latest_Power_Sample(&power_sample);

        Power_Control_Fast_Loop(&power_sample);
        fast_faults = Power_Control_Get_Fault_Status();
        if(fast_faults != 0U && fast_faults != reported_faults) {
            Safety_Manager_Report_Faults(fast_faults);
            reported_faults = fast_faults;
        } else if(fast_faults == 0U) {
            reported_faults = 0U;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(FAST_CONTROL_PERIOD_MS));
    }
}

static void App_Control_Task(void *argument)
{
    TickType_t last_wake;

    (void)argument;

    /* vTaskDelayUntil() 需要保存上一次唤醒 tick，用来保持固定周期。 */
    last_wake = xTaskGetTickCount();
    for(;;) {
        App_Collect_And_Publish_Status(CONTROL_PERIOD_MS);

        /*
         * 使用 vTaskDelayUntil() 而不是 vTaskDelay()：
         * 前者按绝对 tick 对齐周期，可减少任务执行时间变化带来的长期漂移。
         */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

/*
 * 通信任务，周期 10 ms。
 *
 * Bms_Comm_Service_Poll() 内部会完成两件事：
 * - 读走 UART 已收到的字节并喂给协议解析器；
 * - 按 100 ms/500 ms 周期自动发送实时状态帧和单体电压帧。
 *
 * 通信任务优先级仅低于安全任务。UART RX/TX 使用中断环形缓冲，
 * Bms_Comm_Service_Poll() 本身不等待字节发送完成。
 */
static void App_Comm_Task(void *argument)
{
    TickType_t last_wake;

    (void)argument;

    last_wake = xTaskGetTickCount();
    for(;;) {
        Bms_Comm_Service_Poll(Board_Support_Millis());
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(COMM_PERIOD_MS));
    }
}

static void App_Heartbeat_Task(void *argument)
{
    (void)argument;

    for(;;) {
        /* LED1 心跳闪烁：只要它还在翻转，就说明调度器和低优先级任务仍在运行。 */
        Board_Support_Led_Toggle();
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_PERIOD_MS));
    }
}

void App_Tasks_Start(void)
{
    /*
     * 初始化顺序需要固定：
     * - 先准备共享状态，再创建任务；
     * - 先初始化底层驱动，再初始化会调用驱动的管理模块；
     * - 首次 AFE/ADC 采样放到调度器启动后的控制任务中执行，保证 I2C 互斥和恢复逻辑有效。
     */
    Bms_State_Init();
    /*
     * Bring up UART before board-specific peripherals so a serial terminal can
     * still see a boot/status frame if a later AFE or power-stage init stalls.
     */
    Bms_Comm_Service_Init();
    Param_Storage_Init();
    Fault_Log_Init();
    Afe_Gd30bm2016_Init();
    Adc_Manager_Init();
    Power_Control_Init();
    Safety_Manager_Init();
    Power_Path_Manager_Init();
    Power_Manager_Init();
    Charge_Manager_Init();
    Balance_Manager_Init();
    Soc_Estimator_Init();

    /*
     * 安全任务优先级最高，任何硬件故障先走这里关断 Q4/Q5。
     */
    configASSERT(xTaskCreate(App_Safety_Task,
                             "bms_safety",
                             256,
                             0,
                             APP_TASK_PRIORITY_SAFETY,
                             0) == pdPASS);

    /*
     * 通信任务优先级次高，确保上位机实时收发不被普通控制逻辑长期占用。
     */
    configASSERT(xTaskCreate(App_Fast_Control_Task,
                             "bms_fast",
                             256,
                             0,
                             APP_TASK_PRIORITY_FAST_CONTROL,
                             0) == pdPASS);

    configASSERT(xTaskCreate(App_Comm_Task,
                             "bms_comm",
                             512,
                             0,
                             APP_TASK_PRIORITY_COMM,
                             0) == pdPASS);

    /*
     * 控制任务栈空间相对更大，因为它持有 AFE、ADC、status 等结构体。
     * 这里没有保存任务句柄，当前版本不需要外部挂起/恢复这些任务。
     */
    configASSERT(xTaskCreate(App_Control_Task,
                             "bms_ctrl",
                             512,
                             0,
                             APP_TASK_PRIORITY_CONTROL,
                             0) == pdPASS);

    /* 心跳任务最低优先级，用于观察系统是否还有空闲调度能力。 */
    configASSERT(xTaskCreate(App_Heartbeat_Task,
                             "heartbeat",
                             128,
                             0,
                             APP_TASK_PRIORITY_HEARTBEAT,
                             0) == pdPASS);
}
