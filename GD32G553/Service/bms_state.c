#include "bms_state.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

/*
 * 全局状态快照。
 *
 * 控制任务负责写入，通信任务负责读取。使用 FreeRTOS 临界区保护结构体拷贝，
 * 避免 UART 任务打包到更新了一半的电池状态帧。
 */
static bms_status_t s_status;

static void Bms_State_Fill_Defaults(bms_status_t *status)
{
    memset(status, 0, sizeof(*status));

    /*
     * 正式固件默认快照不填任何模拟电芯、电压、温度或 SOC。
     * 如果通信任务在首轮真实采样前被调度，上位机会看到“数据为 0 + 故障位”，
     * 而不是看起来正常的假电池包。
     */
    status->faultBitmap = BMS_FAULT_AFE_COMM | BMS_FAULT_ADC;
    status->chargeState = (uint8_t)BMS_CHARGE_STATE_IDLE;
    status->chargeMode = (uint8_t)BMS_CHARGE_MODE_AUTO;
    status->workMode = (uint8_t)BMS_WORK_MODE_BMS;
}

void Bms_State_Init(void)
{
    taskENTER_CRITICAL();
    Bms_State_Fill_Defaults(&s_status);
    taskEXIT_CRITICAL();
}

void Bms_State_Set_Status(const bms_status_t *status)
{
    if(status == 0) {
        return;
    }

    /* 结构体拷贝虽然很短，但字段较多，仍然放在临界区内。 */
    taskENTER_CRITICAL();
    s_status = *status;
    taskEXIT_CRITICAL();
}

void Bms_State_Get_Status(bms_status_t *status)
{
    if(status == 0) {
        return;
    }

    /* 读取者拿到稳定快照后，再在临界区外执行协议序列化。 */
    taskENTER_CRITICAL();
    *status = s_status;
    taskEXIT_CRITICAL();
}
