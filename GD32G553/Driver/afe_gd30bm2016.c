#include "afe_gd30bm2016.h"

#include "afe_gd30bm2016_regs.h"
#include "bms_board_config.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include <string.h>


#define AFE_I2C_DELAY_CYCLES                  5000U
#define AFE_1MS_DELAY_CYCLES                  120000U
#define AFE_RECOVER_PERIOD_MS                 500U
#define AFE_I2C_CRC_FRAME_MAX                 8U

#if BMS_CELL_COUNT != 9U
#error "GD30BM2016 tested I2C mapping in this file is for the 9S main board."
#endif

typedef struct {
    uint16_t address;
    uint32_t value;
    uint8_t length;
} gd30_init_write_t;

#define GD30BM2016_INIT_ACCESS_KEY_1          0x5A5A5A5AU
#define GD30BM2016_INIT_ACCESS_KEY_2        0xA5A5A5A5U
/*
 * 主控板 9S 接法：
 * - Cell1..Cell8 = VC1-VC0 .. VC8-VC7
 * - Cell9        = VC16-VC15
 * - VC9..VC15 短到 VC8，所以 BM2016 的 Cell9..Cell15 直接读数为 0 mV 是设计结果。
 *
 * VCellMode 0x807F 启用低 8 节和高位 Cell16 通道；中间短接通道不启用，
 * 避免 Cell9..Cell15 的 0V 触发误保护。
 */
#define GD30BM2016_BOARD_LOW_CELL_COUNT     8U
#define GD30BM2016_BOARD_TOP_LOGICAL_MASK   (1U << (BMS_CELL_COUNT - 1U))
#define GD30BM2016_BOARD_TOP_HW_MASK        0x8000U
#define GD30BM2016_INIT_VCELL_MODE_9S       0x807FU
#define GD30BM2016_R71_RSENSE_MOHM          5U
#define GD30BM2016_CC_GAIN_VALUE            (30000U / GD30BM2016_R71_RSENSE_MOHM)

/*
 * 第9串电压获取策略：
 * 0 = 读取 CELL16_VOLTAGE 寄存器（9S 高位通道）
 * 1 = 用 StackVoltage - sum(Cell1..Cell8) 推算（适合高位 VC 短接或映射不确定时）
 *
 * 上板验证建议：对比 g_afe_gd30_raw_cell16_mv 和 g_afe_gd30_stack_minus_cell1_8_mv
 * 与万用表实测第9串电压。
 */
#define GD30BM2016_CELL9_SOURCE_DERIVED     0U

#define GD30BM2016_INIT_PROTECTION_CONFIG     0x0602U
#define GD30BM2016_INIT_COMM_IDLE_TIME        0x0AU
#define GD30BM2016_INIT_CUV_THRESHOLD         0x0088U
#define GD30BM2016_INIT_COV_THRESHOLD         0x00D1U
#define GD30BM2016_INIT_BAL_MIN_CELL_MV       0x0E42U
#define GD30BM2016_INIT_BAL_DELTA_START       0x28U
#define GD30BM2016_INIT_BAL_DELTA_STOP        0x14U
/*
 * 充电器场景关闭 pdsg_en：本板 LD/PACK 检测点都接在变换器输出侧(VIN+/Vout)，
 * 预放电门闩 = "Vout≈Stack≤100mV"。串联同口合主路径(CHG+DSG)时，pdsg_en=1 会
 * 让芯片先跑预放电；handover 切到 normal FET、电阻钳位消失的瞬间 boost 空载兜不住
 * Vout，门闩永不满足 → 主 DSG 不 latch → 主路径断开 → Vout 崩塌重试。预放电是为
 * 接负载抑制浪涌设计的，充电器不需要，故去掉 PDSG_EN。
 */
#define GD30BM2016_INIT_FET_OPTIONS           (GD30BM2016_FET_OPTIONS_DEFAULT)
#define GD30BM2016_INIT_PRECHARGE_START_MV    3000U
#define GD30BM2016_INIT_PRECHARGE_STOP_MV     3200U
#define GD30BM2016_INIT_PREDISCHARGE_TIMEOUT  5U
#define GD30BM2016_INIT_PREDISCHARGE_DELTA_MV 100U
#define GD30BM2016_FET_CONTROL_DSG_OFF        (1U << 0)
#define GD30BM2016_FET_CONTROL_PDSG_OFF       (1U << 1)
#define GD30BM2016_FET_CONTROL_CHG_OFF        (1U << 2)
#define GD30BM2016_FET_CONTROL_PCHG_OFF       (1U << 3)
#define GD30BM2016_FET_CONTROL_ALL_OFF        (GD30BM2016_FET_CONTROL_DSG_OFF |  \
                                               GD30BM2016_FET_CONTROL_PDSG_OFF | \
                                               GD30BM2016_FET_CONTROL_CHG_OFF |  \
                                               GD30BM2016_FET_CONTROL_PCHG_OFF)
#define GD30BM2016_MANUFACTURING_STATUS_FET_EN (1U << 4)
#define GD30BM2016_FET_STATUS_OUTPUT_MASK     (AFE_GD30BM2016_FET_STATUS_CHG |  \
                                               AFE_GD30BM2016_FET_STATUS_PCHG | \
                                               AFE_GD30BM2016_FET_STATUS_DSG |  \
                                               AFE_GD30BM2016_FET_STATUS_PDSG)
#define GD30BM2016_MAIN_PATH_FET_MASK         (AFE_GD30BM2016_FET_STATUS_CHG |  \
                                               AFE_GD30BM2016_FET_STATUS_DSG)
#define GD30BM2016_PRECONNECT_FET_MASK        (AFE_GD30BM2016_FET_STATUS_PCHG | \
                                               AFE_GD30BM2016_FET_STATUS_PDSG)
#define GD30BM2016_FET_ENABLE_RETRY_COUNT     3U
#define GD30BM2016_FET_ENABLE_RETRY_DELAY_MS  20U

static SemaphoreHandle_t s_i2c_mutex;
static uint8_t s_i2c_ready;
static uint8_t s_i2c_addr_write;
static TickType_t s_last_recover_tick;
static uint16_t s_balance_bitmap;

volatile uint8_t g_afe_gd30_config_fail_stage;
volatile uint8_t g_afe_gd30_config_fail_step;
volatile uint8_t g_afe_gd30_config_fail_index;
volatile uint8_t g_afe_gd30_cfgupdate_seen;
volatile uint8_t g_afe_gd30_i2c_addr_write;

/* 调试变量：上板验证 9S 映射和 VCellMode 配置 */
volatile uint16_t g_afe_gd30_probe_vcell_mode;
volatile uint16_t g_afe_gd30_raw_cell9_mv;
volatile uint16_t g_afe_gd30_raw_cell16_mv;
volatile uint16_t g_afe_gd30_stack_minus_cell1_8_mv;
volatile uint8_t g_afe_gd30_i2c_last_reg;
volatile uint8_t g_afe_gd30_i2c_last_crc_ok;
volatile uint8_t g_afe_gd30_i2c_last_crc_rx;
volatile uint8_t g_afe_gd30_i2c_last_crc_calc;
volatile uint16_t g_afe_gd30_config_fail_reg;
volatile uint16_t g_afe_gd30_last_battery_status;

enum {
    GD30_CFG_FAIL_NONE = 0U,
    GD30_CFG_FAIL_SEAL = 1U,
    GD30_CFG_FAIL_KEY1 = 2U,
    GD30_CFG_FAIL_KEY2 = 3U,
    GD30_CFG_FAIL_SET_CFGUPDATE = 4U,
    GD30_CFG_FAIL_WAIT_CFGUPDATE = 5U,
    GD30_CFG_FAIL_INIT_TABLE = 6U,
    GD30_CFG_FAIL_EXIT_CFGUPDATE = 7U,
    GD30_CFG_FAIL_SLEEP_DISABLE = 8U,
    GD30_CFG_FAIL_PROBE = 9U,
    GD30_CFG_FAIL_FET_ENABLE = 10U
};

enum {
    GD30_CFG_FAIL_STEP_NONE = 0U,
    GD30_CFG_FAIL_STEP_CMD = 1U,
    GD30_CFG_FAIL_STEP_META = 2U,
    GD30_CFG_FAIL_STEP_DATA = 3U,
    GD30_CFG_FAIL_STEP_READ_LEN = 4U,
    GD30_CFG_FAIL_STEP_READ_DATA = 5U
};

static const uint8_t s_cell_voltage_regs[BMS_CELL_COUNT] = {
    GD30BM2016_DIR_CELL1_VOLTAGE,
    GD30BM2016_DIR_CELL2_VOLTAGE,
    GD30BM2016_DIR_CELL3_VOLTAGE,
    GD30BM2016_DIR_CELL4_VOLTAGE,
    GD30BM2016_DIR_CELL5_VOLTAGE,
    GD30BM2016_DIR_CELL6_VOLTAGE,
    GD30BM2016_DIR_CELL7_VOLTAGE,
    GD30BM2016_DIR_CELL8_VOLTAGE,
    GD30BM2016_DIR_CELL16_VOLTAGE
};

static const uint8_t s_temperature_regs[BMS_AFE_TEMP_COUNT] = {
    GD30BM2016_DIR_INT_TEMPERATURE,
    GD30BM2016_DIR_TS1_TEMPERATURE,
    GD30BM2016_DIR_TS2_TEMPERATURE,
    GD30BM2016_DIR_TS3_TEMPERATURE
};

static const gd30_init_write_t s_gd30_init_table[] = {
    {GD30BM2016_DM_COMM_IDLE_TIME, GD30BM2016_INIT_COMM_IDLE_TIME, 1U},
    {GD30BM2016_DM_VCELL_MODE, GD30BM2016_INIT_VCELL_MODE_9S, 2U},
    {GD30BM2016_DM_PROTECTION_CONFIGURATION, GD30BM2016_INIT_PROTECTION_CONFIG, 2U},
    {GD30BM2016_DM_ENABLED_PROTECTIONS_A, GD30BM2016_SAFETY_A_CRITICAL_MASK, 1U},
    {GD30BM2016_DM_ENABLED_PROTECTIONS_B, GD30BM2016_SAFETY_B_OVERTEMP_MASK, 1U},
    {GD30BM2016_DM_ENABLED_PROTECTIONS_C, GD30BM2016_SAFETY_C_CRITICAL_MASK, 1U},
    {GD30BM2016_DM_BODY_DIODE_THRESHOLD, 0x03E8U, 2U},
    {GD30BM2016_DM_FET_OPTIONS, GD30BM2016_INIT_FET_OPTIONS, 1U},
    {GD30BM2016_DM_CHG_PUMP_CONTROL, GD30BM2016_DEMO_CHG_PUMP_CONTROL, 1U},
    {GD30BM2016_DM_PRECHARGE_START_VOLTAGE, GD30BM2016_INIT_PRECHARGE_START_MV, 2U},
    {GD30BM2016_DM_PRECHARGE_STOP_VOLTAGE, GD30BM2016_INIT_PRECHARGE_STOP_MV, 2U},
    {GD30BM2016_DM_PREDISCHARGE_TIMEOUT, GD30BM2016_INIT_PREDISCHARGE_TIMEOUT, 1U},
    {GD30BM2016_DM_PREDISCHARGE_STOP_DELTA, GD30BM2016_INIT_PREDISCHARGE_DELTA_MV, 1U},
    {GD30BM2016_DM_CUV_THRESHOLD, GD30BM2016_INIT_CUV_THRESHOLD, 2U},
    {GD30BM2016_DM_CUV_DELAY, 0x0400U, 2U},
    {GD30BM2016_DM_CUV_RECOVERY_HYSTERESIS, 0x0EU, 1U},
    {GD30BM2016_DM_COV_THRESHOLD, GD30BM2016_INIT_COV_THRESHOLD, 2U},
    {GD30BM2016_DM_COV_DELAY, 0x02D5U, 2U},
    {GD30BM2016_DM_COV_RECOVERY_HYSTERESIS, 0x05U, 1U},
    {GD30BM2016_DM_COVL_LATCH_LIMIT, 0x01U, 1U},
    {GD30BM2016_DM_COVL_COUNTER_DEC_DELAY, 0x02U, 1U},
    {GD30BM2016_DM_COVL_RECOVERY_TIME, 0x04U, 1U},
    {GD30BM2016_DM_OCC_THRESHOLD, 0x0005U, 2U},
    {GD30BM2016_DM_OCC_DELAY, 0xFDU, 1U},
    {GD30BM2016_DM_OCC_RECOVERY_THRESHOLD, 0x0708U, 2U},
    {GD30BM2016_DM_OCD1_THRESHOLD, 0x0003U, 2U},
    {GD30BM2016_DM_OCD1_DELAY, 0xFDU, 1U},
    {GD30BM2016_DM_OCD2_THRESHOLD, 0x0007U, 2U},
    {GD30BM2016_DM_OCD2_DELAY, 0x7EU, 1U},
    {GD30BM2016_DM_OCD_RECOVERY_THRESHOLD, 0xFF00U, 2U},
    {GD30BM2016_DM_SCD_THRESHOLD, 0x02U, 1U},
    {GD30BM2016_DM_SCD_DELAY, 0x02U, 1U},
    {GD30BM2016_DM_SCD_RECOVERY_TIME, 0x05U, 1U},
    {GD30BM2016_DM_SCDL_LATCH_LIMIT, 0x02U, 1U},
    {GD30BM2016_DM_SCDL_COUNTER_DEC_DELAY, 0x0AU, 1U},
    {GD30BM2016_DM_SCDL_RECOVERY_TIME, 0x0AU, 1U},
    {GD30BM2016_DM_SCDL_RECOVERY_THRESHOLD, 0x2710U, 2U},
    {GD30BM2016_DM_OTC_THRESHOLD, 0x3CU, 1U},
    {GD30BM2016_DM_OTC_DELAY, 0x05U, 1U},
    {GD30BM2016_DM_OTC_RECOVERY_THRESHOLD, 0x2DU, 1U},
    {GD30BM2016_DM_OTD_THRESHOLD, 0x41U, 1U},
    {GD30BM2016_DM_OTD_DELAY, 0x05U, 1U},
    {GD30BM2016_DM_OTD_RECOVERY_THRESHOLD, 0x32U, 1U},
    {GD30BM2016_DM_UTC_THRESHOLD, 0xFBU, 1U},
    {GD30BM2016_DM_UTC_DELAY, 0x02U, 1U},
    {GD30BM2016_DM_UTC_RECOVERY_THRESHOLD, 0x01U, 1U},
    {GD30BM2016_DM_UTD_THRESHOLD, 0xD3U, 1U},
    {GD30BM2016_DM_UTD_DELAY, 0x02U, 1U},
    {GD30BM2016_DM_UTD_RECOVERY_THRESHOLD, 0xD8U, 1U},
    {GD30BM2016_DM_UTINT_RECOVERY_THRESHOLD, 0xF1U, 1U},
    {GD30BM2016_DM_RECOVERY_TIME, 0x0AU, 1U},
    {GD30BM2016_DM_DDSG_PIN_CONFIG, GD30BM2016_PIN_FXN_STATUS, 1U},
    {GD30BM2016_DM_TS1_CONFIG, 0x50U, 1U},
    {GD30BM2016_DM_TS23_CONFIG, 0x55U, 1U},
    {GD30BM2016_DM_PAD_CFG, 0x90U, 1U},
    {GD30BM2016_DM_CC_GAIN, GD30BM2016_CC_GAIN_VALUE, 2U},
    {GD30BM2016_DM_T18K_COEFF_A1, 0xEC67U, 2U},
    {GD30BM2016_DM_T18K_COEFF_A2, 0x4262U, 2U},
    {GD30BM2016_DM_T18K_COEFF_A3, 0xF6D7U, 2U},
    {GD30BM2016_DM_T18K_COEFF_A4, 0x04C4U, 2U},
    {GD30BM2016_DM_POWER_CONFIG, 0x4F86U, 2U},
    {GD30BM2016_DM_REG12_CONFIG, GD30BM2016_DEMO_REG12_CONFIG, 1U},
    {GD30BM2016_DM_REG0_CONFIG, GD30BM2016_DEMO_REG0_CONFIG, 1U},
    {GD30BM2016_DM_COMM_IDLE_TIME, GD30BM2016_INIT_COMM_IDLE_TIME, 1U},
    {GD30BM2016_DM_MANUFACTURING, GD30BM2016_DEMO_MANUFACTURING_FET_EN, 1U},
    {GD30BM2016_DM_SLEEP_CHARGER_VOLTAGE_THRESHOLD, 0x1F40U, 2U},
    {GD30BM2016_DM_ADC_CLK_CFG, 0x00U, 1U},
    {GD30BM2016_DM_BALANCE_CONFIG, GD30BM2016_DEMO_BALANCE_CONFIG, 1U},
    {GD30BM2016_DM_BAL_MIN_CELL_TEMP, 0xECU, 1U},
    {GD30BM2016_DM_BAL_MAX_CELL_TEMP, 0x3CU, 1U},
    {GD30BM2016_DM_BAL_MAX_INTERNAL_TEMP, 0x46U, 1U},
    {GD30BM2016_DM_BAL_CELL_INTERVAL, 0x05U, 1U},
    {GD30BM2016_DM_BAL_MAX_CELLS, 0x03U, 1U},
    {GD30BM2016_DM_BAL_MIN_CELL_V_CHARGE, GD30BM2016_INIT_BAL_MIN_CELL_MV, 2U},
    {GD30BM2016_DM_BAL_MIN_DELTA_CHARGE, GD30BM2016_INIT_BAL_DELTA_START, 1U},
    {GD30BM2016_DM_BAL_STOP_DELTA_CHARGE, GD30BM2016_INIT_BAL_DELTA_STOP, 1U},
    {GD30BM2016_DM_BAL_MIN_CELL_V_RELAX, GD30BM2016_INIT_BAL_MIN_CELL_MV, 2U},
    {GD30BM2016_DM_BAL_MIN_DELTA_RELAX, GD30BM2016_INIT_BAL_DELTA_START, 1U},
    {GD30BM2016_DM_BAL_STOP_DELTA_RELAX, GD30BM2016_INIT_BAL_DELTA_STOP, 1U},
    {GD30BM2016_DM_LOAD_DETECT_ACTIVE_TIME, 0x0AU, 1U},
    {GD30BM2016_DM_LOAD_DETECT_RETRY_DELAY, 0x0FU, 1U},
    {GD30BM2016_DM_LOAD_DETECT_TIMEOUT, 0xFFFFU, 2U}
};

static uint16_t Gd30_Balance_Logical_To_Hw(uint16_t logical_bitmap)
{
    uint16_t hw_bitmap;

    hw_bitmap = (uint16_t)(logical_bitmap & ((1U << GD30BM2016_BOARD_LOW_CELL_COUNT) - 1U));
    if((logical_bitmap & GD30BM2016_BOARD_TOP_LOGICAL_MASK) != 0U) {
        hw_bitmap |= GD30BM2016_BOARD_TOP_HW_MASK;
    }

    return hw_bitmap;
}

static int16_t Gd30_Sanitize_Temperature_X10(int16_t temperature_x10)
{
    if(temperature_x10 == (int16_t)BMS_TEMP_UNAVAILABLE_X10) {
        return (int16_t)BMS_TEMP_UNAVAILABLE_X10;
    }
    if(temperature_x10 < (int16_t)BMS_TEMP_MIN_VALID_X10 ||
       temperature_x10 > (int16_t)BMS_TEMP_MAX_VALID_X10) {
        return (int16_t)BMS_TEMP_UNAVAILABLE_X10;
    }

    return temperature_x10;
}

static uint8_t Gd30_I2c_Lock(TickType_t timeout)
{
    if(s_i2c_mutex == 0) {
        return 1U;
    }
    if(xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return 1U;
    }

    return (xSemaphoreTake(s_i2c_mutex, timeout) == pdTRUE) ? 1U : 0U;
}

static void Gd30_I2c_Unlock(void)
{
    if((s_i2c_mutex != 0) &&
       (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)) {
        (void)xSemaphoreGive(s_i2c_mutex);
    }
}

static void Afe_Delay_Cycles(uint32_t cycles)
{
    volatile uint32_t i;

    for(i = 0U; i < cycles; i++) {
        __NOP();
    }
}

static void Afe_Delay_Ms(uint32_t ms)
{
    uint32_t i;

    if((ms != 0U) &&
       (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)) {
        vTaskDelay(pdMS_TO_TICKS(ms));
        return;
    }

    for(i = 0U; i < ms; i++) {
        Afe_Delay_Cycles(AFE_1MS_DELAY_CYCLES);
    }
}

static void Afe_I2c_Delay(void)
{
    Afe_Delay_Cycles(AFE_I2C_DELAY_CYCLES);
}

static void Afe_I2c_Scl_High(void)
{
    gpio_bit_set(BMS_AFE_I2C_GPIO_PORT, BMS_AFE_I2C_SCL_PIN);
}

static void Afe_I2c_Scl_Low(void)
{
    gpio_bit_reset(BMS_AFE_I2C_GPIO_PORT, BMS_AFE_I2C_SCL_PIN);
}

static void Afe_I2c_Sda_Release(void)
{
    gpio_bit_set(BMS_AFE_I2C_GPIO_PORT, BMS_AFE_I2C_SDA_PIN);
    gpio_mode_set(BMS_AFE_I2C_GPIO_PORT,
                  GPIO_MODE_INPUT,
                  GPIO_PUPD_PULLUP,
                  BMS_AFE_I2C_SDA_PIN);
}

static void Afe_I2c_Sda_Drive_Low(void)
{
    gpio_bit_reset(BMS_AFE_I2C_GPIO_PORT, BMS_AFE_I2C_SDA_PIN);
    gpio_mode_set(BMS_AFE_I2C_GPIO_PORT,
                  GPIO_MODE_OUTPUT,
                  GPIO_PUPD_NONE,
                  BMS_AFE_I2C_SDA_PIN);
    gpio_output_options_set(BMS_AFE_I2C_GPIO_PORT,
                            GPIO_OTYPE_OD,
                            GPIO_OSPEED_60MHZ,
                            BMS_AFE_I2C_SDA_PIN);
}

static void Afe_I2c_Sda_High(void)
{
    Afe_I2c_Sda_Release();
}

static void Afe_I2c_Sda_Low(void)
{
    Afe_I2c_Sda_Drive_Low();
}

static uint8_t Afe_I2c_Sda_Is_High(void)
{
    return (RESET != gpio_input_bit_get(BMS_AFE_I2C_GPIO_PORT, BMS_AFE_I2C_SDA_PIN)) ? 1U : 0U;
}

static void Afe_I2c_Start(void)
{
    Afe_I2c_Sda_High();
    Afe_I2c_Scl_High();
    Afe_I2c_Delay();
    Afe_I2c_Sda_Low();
    Afe_I2c_Delay();
    Afe_I2c_Scl_Low();
    Afe_I2c_Delay();
}

static void Afe_I2c_Stop(void)
{
    Afe_I2c_Sda_Low();
    Afe_I2c_Delay();
    Afe_I2c_Scl_High();
    Afe_I2c_Delay();
    Afe_I2c_Sda_High();
    Afe_I2c_Delay();
}

static void Afe_I2c_Bus_Recover(void)
{
    uint8_t i;

    Afe_I2c_Sda_High();
    for(i = 0U; i < 9U; i++) {
        Afe_I2c_Scl_Low();
        Afe_I2c_Delay();
        Afe_I2c_Scl_High();
        Afe_I2c_Delay();
    }
    Afe_I2c_Stop();
}

static uint8_t Afe_I2c_Write_Byte(uint8_t value)
{
    uint8_t i;
    uint8_t ack;

    for(i = 0U; i < 8U; i++) {
        if((value & 0x80U) != 0U) {
            Afe_I2c_Sda_High();
        } else {
            Afe_I2c_Sda_Low();
        }
        Afe_I2c_Delay();
        Afe_I2c_Scl_High();
        Afe_I2c_Delay();
        Afe_I2c_Scl_Low();
        Afe_I2c_Delay();
        value <<= 1U;
    }

    Afe_I2c_Sda_High();
    Afe_I2c_Delay();
    Afe_I2c_Scl_High();
    Afe_I2c_Delay();
    ack = (Afe_I2c_Sda_Is_High() == 0U) ? 1U : 0U;
    Afe_I2c_Scl_Low();
    Afe_I2c_Delay();
    Afe_I2c_Sda_High();
    Afe_I2c_Delay();

    return ack;
}

static uint8_t Afe_I2c_Read_Byte(uint8_t ack)
{
    uint8_t i;
    uint8_t value;

    value = 0U;
    Afe_I2c_Sda_High();
    for(i = 0U; i < 8U; i++) {
        value <<= 1U;
        Afe_I2c_Delay();
        Afe_I2c_Scl_High();
        Afe_I2c_Delay();
        if(Afe_I2c_Sda_Is_High() != 0U) {
            value |= 1U;
        }
        Afe_I2c_Scl_Low();
        Afe_I2c_Delay();
    }

    if(ack != 0U) {
        Afe_I2c_Sda_Low();
    } else {
        Afe_I2c_Sda_High();
    }
    Afe_I2c_Delay();
    Afe_I2c_Scl_High();
    Afe_I2c_Delay();
    Afe_I2c_Scl_Low();
    Afe_I2c_Delay();
    Afe_I2c_Sda_High();

    return value;
}

static void Afe_I2c_Gpio_Init(void)
{
    rcu_periph_clock_enable(BMS_AFE_I2C_GPIO_CLK);
    rcu_periph_clock_enable(BMS_AFE_RST_GPIO_CLK);
    rcu_periph_clock_enable(BMS_AFE_ALERT_GPIO_CLK);
    rcu_periph_clock_enable(BMS_AFE_DDSG_DCHG_GPIO_CLK);
    rcu_periph_clock_enable(BMS_AFE_DFETOFF_GPIO_CLK);

    gpio_bit_reset(BMS_AFE_RST_GPIO_PORT, BMS_AFE_RST_PIN);
    gpio_mode_set(BMS_AFE_RST_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, BMS_AFE_RST_PIN);
    gpio_output_options_set(BMS_AFE_RST_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, BMS_AFE_RST_PIN);
    gpio_bit_reset(BMS_AFE_RST_GPIO_PORT, BMS_AFE_RST_PIN);
    Afe_Delay_Ms(5U);

    gpio_bit_reset(BMS_AFE_I2C_GPIO_PORT, BMS_AFE_I2C_ADDR0_PIN | BMS_AFE_I2C_ADDR1_PIN);
    gpio_mode_set(BMS_AFE_I2C_GPIO_PORT,
                  GPIO_MODE_OUTPUT,
                  GPIO_PUPD_PULLDOWN,
                  BMS_AFE_I2C_ADDR0_PIN | BMS_AFE_I2C_ADDR1_PIN);
    gpio_output_options_set(BMS_AFE_I2C_GPIO_PORT,
                            GPIO_OTYPE_PP,
                            GPIO_OSPEED_60MHZ,
                            BMS_AFE_I2C_ADDR0_PIN | BMS_AFE_I2C_ADDR1_PIN);

    gpio_bit_set(BMS_AFE_I2C_GPIO_PORT, BMS_AFE_I2C_SCL_PIN | BMS_AFE_I2C_SDA_PIN);
    gpio_mode_set(BMS_AFE_I2C_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, BMS_AFE_I2C_SCL_PIN);
    gpio_output_options_set(BMS_AFE_I2C_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, BMS_AFE_I2C_SCL_PIN);
    Afe_I2c_Sda_Release();
    Afe_I2c_Bus_Recover();

    gpio_mode_set(BMS_AFE_ALERT_GPIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, BMS_AFE_ALERT_PIN);
    gpio_mode_set(BMS_AFE_DDSG_DCHG_GPIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, BMS_AFE_DDSG_PIN | BMS_AFE_DCHG_PIN);
    gpio_bit_reset(BMS_AFE_DFETOFF_GPIO_PORT, BMS_AFE_DFETOFF_PIN);
    gpio_mode_set(BMS_AFE_DFETOFF_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, BMS_AFE_DFETOFF_PIN);
    gpio_output_options_set(BMS_AFE_DFETOFF_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, BMS_AFE_DFETOFF_PIN);
}

static uint8_t Afe_I2c_Write_Raw(uint8_t address, uint8_t reg_addr, const uint8_t *data, uint8_t length)
{
    uint8_t ok;

    if((data == 0) || (length == 0U)) {
        return 0U;
    }

    ok = 1U;
    Afe_I2c_Start();
    ok = Afe_I2c_Write_Byte(address);
    if(ok != 0U) {
        ok = Afe_I2c_Write_Byte(reg_addr);
    }
    while((length-- != 0U) && (ok != 0U)) {
        ok = Afe_I2c_Write_Byte(*data);
        data++;
    }
    Afe_I2c_Stop();

    return ok;
}

static uint8_t Afe_I2c_Read_Raw(uint8_t address, uint8_t reg_addr, uint8_t *data, uint8_t length)
{
    uint8_t ok;
    uint8_t i;

    if((data == 0) || (length == 0U)) {
        return 0U;
    }

    ok = 1U;
    Afe_I2c_Start();
    ok = Afe_I2c_Write_Byte(address);
    if(ok != 0U) {
        ok = Afe_I2c_Write_Byte(reg_addr);
    }
    if(ok != 0U) {
        Afe_I2c_Start();
        ok = Afe_I2c_Write_Byte((uint8_t)(address | 0x01U));
    }
    for(i = 0U; (i < length) && (ok != 0U); i++) {
        data[i] = Afe_I2c_Read_Byte((i + 1U) < length);
    }
    Afe_I2c_Stop();

    if(ok == 0U) {
        memset(data, 0, length);
    }

    return ok;
}

static uint8_t Gd30_I2c_Crc8(const uint8_t *data, uint8_t length)
{
    uint8_t crc;
    uint8_t i;
    uint8_t value;

    crc = GD30BM2016_CRC8_INIT;
    while(length-- != 0U) {
        value = *data;
        data++;
        for(i = 0x80U; i != 0U; i >>= 1U) {
            if((crc & 0x80U) != 0U) {
                crc = (uint8_t)((crc << 1U) ^ GD30BM2016_CRC8_POLY);
            } else {
                crc = (uint8_t)(crc << 1U);
            }
            if((value & i) != 0U) {
                crc ^= GD30BM2016_CRC8_POLY;
            }
        }
    }

    return crc;
}

static uint8_t Gd30_Direct_Write(uint8_t address, const uint8_t *data, uint8_t length)
{
    uint8_t crc_input[AFE_I2C_CRC_FRAME_MAX];
    uint8_t tx[AFE_I2C_CRC_FRAME_MAX];
    uint8_t i;

    if((data == 0) || (length == 0U) ||
       ((uint8_t)(length + 1U) > AFE_I2C_CRC_FRAME_MAX) ||
       ((uint8_t)(length + 2U) > AFE_I2C_CRC_FRAME_MAX)) {
        return 0U;
    }

    g_afe_gd30_i2c_last_reg = address;
    g_afe_gd30_i2c_last_crc_ok = 0U;
    g_afe_gd30_i2c_last_crc_rx = 0U;

    crc_input[0] = s_i2c_addr_write;
    crc_input[1] = address;
    for(i = 0U; i < length; i++) {
        crc_input[i + 2U] = data[i];
        tx[i] = data[i];
    }
    tx[length] = Gd30_I2c_Crc8(crc_input, (uint8_t)(length + 2U));
    g_afe_gd30_i2c_last_crc_calc = tx[length];

    if(0U == Afe_I2c_Write_Raw(s_i2c_addr_write, address, tx, (uint8_t)(length + 1U))) {
        if(0U == Afe_I2c_Write_Raw(s_i2c_addr_write, address, data, length)) {
            return 0U;
        }
        g_afe_gd30_i2c_last_crc_ok = 2U;
        return 1U;
    }

    g_afe_gd30_i2c_last_crc_ok = 1U;
    return 1U;
}

static uint8_t Gd30_Direct_Read(uint8_t address, uint8_t *data, uint8_t length)
{
    uint8_t crc_input[AFE_I2C_CRC_FRAME_MAX];
    uint8_t rx[AFE_I2C_CRC_FRAME_MAX];
    uint8_t crc;
    uint8_t i;

    if((data == 0) || (length == 0U) ||
       ((uint8_t)(length + 1U) > AFE_I2C_CRC_FRAME_MAX) ||
       ((uint8_t)(length + 3U) > AFE_I2C_CRC_FRAME_MAX)) {
        return 0U;
    }

    g_afe_gd30_i2c_last_reg = address;
    g_afe_gd30_i2c_last_crc_ok = 0U;

    if(0U == Afe_I2c_Read_Raw(s_i2c_addr_write, address, rx, (uint8_t)(length + 1U))) {
        if(0U == Afe_I2c_Read_Raw(s_i2c_addr_write, address, data, length)) {
            memset(data, 0, length);
            return 0U;
        }
        g_afe_gd30_i2c_last_crc_ok = 2U;
        return 1U;
    }

    crc_input[0] = s_i2c_addr_write;
    crc_input[1] = address;
    crc_input[2] = (uint8_t)(s_i2c_addr_write | 0x01U);
    for(i = 0U; i < length; i++) {
        crc_input[i + 3U] = rx[i];
    }

    crc = Gd30_I2c_Crc8(crc_input, (uint8_t)(length + 3U));
    g_afe_gd30_i2c_last_crc_calc = crc;
    g_afe_gd30_i2c_last_crc_rx = rx[length];
    if(crc != rx[length]) {
        if(0U == Afe_I2c_Read_Raw(s_i2c_addr_write, address, data, length)) {
            memset(data, 0, length);
            return 0U;
        }
        g_afe_gd30_i2c_last_crc_ok = 2U;
        return 1U;
    }

    for(i = 0U; i < length; i++) {
        data[i] = rx[i];
    }

    g_afe_gd30_i2c_last_crc_ok = 1U;
    return 1U;
}

static uint8_t Gd30_Direct_Read_U16(uint8_t address, uint16_t *value)
{
    uint8_t bytes[2];

    if(value == 0) {
        return 0U;
    }
    if(0U == Gd30_Direct_Read(address, bytes, 2U)) {
        return 0U;
    }

    *value = (uint16_t)(((uint16_t)bytes[1] << 8) | bytes[0]);
    return 1U;
}

static uint8_t Gd30_Direct_Read_I16(uint8_t address, int16_t *value)
{
    uint16_t raw;

    if(value == 0) {
        return 0U;
    }
    if(0U == Gd30_Direct_Read_U16(address, &raw)) {
        return 0U;
    }

    *value = (int16_t)raw;
    return 1U;
}

static uint8_t Gd30_Checksum(const uint8_t *data, uint8_t length)
{
    uint8_t i;
    uint32_t sum;

    sum = 0U;
    for(i = 0U; i < length; i++) {
        sum += data[i];
    }
    while(sum > 0xFFU) {
        sum = (sum >> 8) + (sum & 0xFFU);
    }

    return (uint8_t)(0xFFU - sum);
}

static uint8_t Gd30_Subcommand_Only(uint16_t command)
{
    uint8_t tx[2];

    g_afe_gd30_config_fail_reg = command;
    g_afe_gd30_config_fail_step = GD30_CFG_FAIL_STEP_CMD;
    tx[0] = (uint8_t)(command & 0xFFU);
    tx[1] = (uint8_t)((command >> 8U) & 0xFFU);

    if(0U == Gd30_Direct_Write(GD30BM2016_DIR_SUBCMD, tx, 2U)) {
        return 0U;
    }

    g_afe_gd30_config_fail_step = GD30_CFG_FAIL_STEP_NONE;
    return 1U;
}

static uint8_t Gd30_Subcommand_Data_Write(uint16_t command, uint32_t value, uint8_t length)
{
    uint8_t bytes[4];
    uint8_t meta[2];
    uint8_t cmd[2];
    uint8_t i;

    if((length == 0U) || (length > sizeof(bytes))) {
        return 0U;
    }

    cmd[0] = (uint8_t)(command & 0xFFU);
    cmd[1] = (uint8_t)((command >> 8U) & 0xFFU);
    for(i = 0U; i < length; i++) {
        bytes[i] = (uint8_t)((value >> (8U * i)) & 0xFFU);
    }

    g_afe_gd30_config_fail_reg = command;
    g_afe_gd30_config_fail_step = GD30_CFG_FAIL_STEP_CMD;
    if(0U == Gd30_Direct_Write(GD30BM2016_DIR_SUBCMD, cmd, 2U)) {
        return 0U;
    }
    Afe_Delay_Ms(1U);

    meta[0] = Gd30_Checksum(bytes, length);
    meta[1] = (uint8_t)(0x80U | length);
    g_afe_gd30_config_fail_step = GD30_CFG_FAIL_STEP_META;
    if(0U == Gd30_Direct_Write(GD30BM2016_DIR_SUBCMD_CHECKSUM_LEN, meta, 2U)) {
        return 0U;
    }
    Afe_Delay_Ms(1U);

    g_afe_gd30_config_fail_step = GD30_CFG_FAIL_STEP_DATA;
    if(0U == Gd30_Direct_Write(GD30BM2016_DIR_SUBCMD_DATA, bytes, length)) {
        return 0U;
    }
    Afe_Delay_Ms(1U);

    g_afe_gd30_config_fail_step = GD30_CFG_FAIL_STEP_NONE;
    return 1U;
}

static uint8_t Gd30_Data_Memory_Write(uint16_t address, uint32_t value, uint8_t length)
{
    return Gd30_Subcommand_Data_Write(address, value, length);
}

static uint8_t Gd30_Data_Memory_Read(uint16_t address, uint8_t *data, uint8_t length)
{
    uint8_t cmd[2];
    uint8_t read_len;

    if((data == 0) || (length == 0U)) {
        return 0U;
    }

    cmd[0] = (uint8_t)(address & 0xFFU);
    cmd[1] = (uint8_t)((address >> 8U) & 0xFFU);
    g_afe_gd30_config_fail_reg = address;
    g_afe_gd30_config_fail_step = GD30_CFG_FAIL_STEP_CMD;
    if(0U == Gd30_Direct_Write(GD30BM2016_DIR_SUBCMD, cmd, 2U)) {
        return 0U;
    }
    Afe_Delay_Ms(1U);

    read_len = length;
    g_afe_gd30_config_fail_step = GD30_CFG_FAIL_STEP_READ_LEN;
    if(0U == Gd30_Direct_Write(GD30BM2016_DIR_SUBCMD_READ_LEN, &read_len, 1U)) {
        return 0U;
    }
    Afe_Delay_Ms(1U);

    g_afe_gd30_config_fail_step = GD30_CFG_FAIL_STEP_READ_DATA;
    if(0U == Gd30_Direct_Read(GD30BM2016_DIR_SUBCMD_DATA, data, length)) {
        return 0U;
    }
    Afe_Delay_Ms(1U);

    g_afe_gd30_config_fail_step = GD30_CFG_FAIL_STEP_NONE;
    return 1U;
}

static uint8_t Gd30_Read_Battery_Status(uint16_t *status)
{
    return Gd30_Direct_Read_U16(GD30BM2016_DIR_BATTERY_STATUS, status);
}

static uint8_t Gd30_Wait_Cfgupdate(void)
{
    uint8_t retry;
    uint16_t status;

    status = 0U;
    for(retry = 0U; retry < 20U; retry++) {
        Afe_Delay_Ms(5U);
        if(0U != Gd30_Read_Battery_Status(&status)) {
            g_afe_gd30_last_battery_status = status;
            if((status & 0x0001U) != 0U) {
                return 1U;
            }
        }
    }

    return 0U;
}

static uint8_t Gd30_I2c_Select_Address(void)
{
    static const uint8_t s_addr_candidates[] = {
        GD30BM2016_I2C_ADDR_WRITE,
        0x22U,
        0x24U,
        0x26U
    };
    uint32_t i;

    for(i = 0U; i < (sizeof(s_addr_candidates) / sizeof(s_addr_candidates[0])); i++) {
        s_i2c_addr_write = s_addr_candidates[i];
        g_afe_gd30_i2c_addr_write = s_i2c_addr_write;
        if(0U != Gd30_Subcommand_Only(GD30BM2016_SUBCMD_SLEEP_DISABLE)) {
            Afe_Delay_Ms(1U);
            return 1U;
        }
    }

    s_i2c_addr_write = GD30BM2016_I2C_ADDR_WRITE;
    return 0U;
}

static uint8_t Gd30_Write_Init_Table(void)
{
    uint32_t i;

    for(i = 0U; i < (sizeof(s_gd30_init_table) / sizeof(s_gd30_init_table[0])); i++) {
        g_afe_gd30_config_fail_index = (uint8_t)i;
        g_afe_gd30_config_fail_reg = s_gd30_init_table[i].address;
        if(0U == Gd30_Data_Memory_Write(s_gd30_init_table[i].address,
                                        s_gd30_init_table[i].value,
                                        s_gd30_init_table[i].length)) {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t Gd30_Enter_Config_Update(void)
{
    g_afe_gd30_config_fail_reg = GD30BM2016_SUBCMD_SEAL;
    g_afe_gd30_config_fail_stage = GD30_CFG_FAIL_SEAL;
    if(0U == Gd30_Subcommand_Only(GD30BM2016_SUBCMD_SEAL)) {
        return 0U;
    }
    Afe_Delay_Ms(1U);

    g_afe_gd30_config_fail_reg = GD30BM2016_CMD_ACCESS_KEYSTEP1;
    g_afe_gd30_config_fail_stage = GD30_CFG_FAIL_KEY1;
    if(0U == Gd30_Subcommand_Data_Write(GD30BM2016_CMD_ACCESS_KEYSTEP1,
                                        GD30BM2016_INIT_ACCESS_KEY_1,
                                        4U)) {
        return 0U;
    }
    g_afe_gd30_config_fail_reg = GD30BM2016_CMD_ACCESS_KEYSTEP1;
    g_afe_gd30_config_fail_stage = GD30_CFG_FAIL_KEY2;
    if(0U == Gd30_Subcommand_Data_Write(GD30BM2016_CMD_ACCESS_KEYSTEP1,
                                        GD30BM2016_INIT_ACCESS_KEY_2,
                                        4U)) {
        return 0U;
    }
    g_afe_gd30_config_fail_reg = GD30BM2016_SUBCMD_SET_CFGUPDATE;
    g_afe_gd30_config_fail_stage = GD30_CFG_FAIL_SET_CFGUPDATE;
    if(0U == Gd30_Subcommand_Only(GD30BM2016_SUBCMD_SET_CFGUPDATE)) {
        return 0U;
    }

    g_afe_gd30_config_fail_stage = GD30_CFG_FAIL_WAIT_CFGUPDATE;
    g_afe_gd30_cfgupdate_seen = Gd30_Wait_Cfgupdate();

    g_afe_gd30_config_fail_stage = GD30_CFG_FAIL_NONE;
    return 1U;
}

static uint8_t Gd30_Exit_Config_Update(void)
{
    if(0U == Gd30_Subcommand_Only(GD30BM2016_SUBCMD_EXIT_CFGUPDATE)) {
        return 0U;
    }
    Afe_Delay_Ms(10U);
    g_afe_gd30_cfgupdate_seen = 0U;

    return 1U;
}

static uint8_t Gd30_Configure_Defaults(void)
{
    g_afe_gd30_config_fail_stage = GD30_CFG_FAIL_NONE;
    g_afe_gd30_config_fail_step = GD30_CFG_FAIL_STEP_NONE;
    g_afe_gd30_config_fail_index = 0U;
    g_afe_gd30_cfgupdate_seen = 0U;
    g_afe_gd30_i2c_last_reg = 0U;
    g_afe_gd30_i2c_last_crc_ok = 0U;
    g_afe_gd30_i2c_last_crc_rx = 0U;
    g_afe_gd30_i2c_last_crc_calc = 0U;
    g_afe_gd30_config_fail_reg = 0U;
    g_afe_gd30_last_battery_status = 0U;
    g_afe_gd30_probe_vcell_mode = 0U;
    g_afe_gd30_raw_cell9_mv = 0U;
    g_afe_gd30_raw_cell16_mv = 0U;
    g_afe_gd30_stack_minus_cell1_8_mv = 0U;

    if(0U == Gd30_Enter_Config_Update()) {
        return 0U;
    }

    g_afe_gd30_config_fail_stage = GD30_CFG_FAIL_INIT_TABLE;
    if(0U == Gd30_Write_Init_Table()) {
        (void)Gd30_Exit_Config_Update();
        return 0U;
    }

    g_afe_gd30_config_fail_reg = GD30BM2016_SUBCMD_EXIT_CFGUPDATE;
    g_afe_gd30_config_fail_stage = GD30_CFG_FAIL_EXIT_CFGUPDATE;
    if(0U == Gd30_Exit_Config_Update()) {
        return 0U;
    }

    g_afe_gd30_config_fail_reg = GD30BM2016_SUBCMD_SLEEP_DISABLE;
    g_afe_gd30_config_fail_stage = GD30_CFG_FAIL_SLEEP_DISABLE;
    if(0U == Gd30_Subcommand_Only(GD30BM2016_SUBCMD_SLEEP_DISABLE)) {
        return 0U;
    }
    Afe_Delay_Ms(1U);

    g_afe_gd30_config_fail_stage = GD30_CFG_FAIL_NONE;
    g_afe_gd30_config_fail_step = GD30_CFG_FAIL_STEP_NONE;
    return 1U;
}

static uint8_t Gd30_Probe_Ready(void)
{
    uint8_t vcell_mode_bytes[2];
    uint16_t battery_status;
    uint16_t vcell_mode;

    vcell_mode_bytes[0] = 0U;
    vcell_mode_bytes[1] = 0U;
    battery_status = 0U;
    vcell_mode = 0U;

    g_afe_gd30_config_fail_reg = GD30BM2016_DIR_BATTERY_STATUS;
    if(0U == Gd30_Read_Battery_Status(&battery_status)) {
        return 0U;
    }
    g_afe_gd30_last_battery_status = battery_status;

    if(0U != Gd30_Data_Memory_Read(GD30BM2016_DM_VCELL_MODE, vcell_mode_bytes, 2U)) {
        vcell_mode = (uint16_t)(((uint16_t)vcell_mode_bytes[1] << 8U) | vcell_mode_bytes[0]);
        g_afe_gd30_probe_vcell_mode = vcell_mode;
    }
    g_afe_gd30_config_fail_step = GD30_CFG_FAIL_STEP_NONE;

    /*
     * The verified GD32G553_BM2016_I2C project does not require a Data Memory
     * readback match before it starts direct voltage reads. Some chips do not
     * report CFGUPDATE in BatteryStatus even though the direct register bus is
     * usable, so the robust readiness gate here is direct-register access.
     */
    return 1U;
}

static uint8_t Gd30_Start_I2c_Link(void)
{
    Afe_I2c_Gpio_Init();
    if(0U == Gd30_I2c_Select_Address()) {
        return 0U;
    }

    if(0U == Gd30_Subcommand_Only(GD30BM2016_SUBCMD_SLEEP_DISABLE)) {
        return 0U;
    }
    Afe_Delay_Ms(1U);
    if(0U == Gd30_Subcommand_Only(GD30BM2016_SUBCMD_SLEEP_DISABLE)) {
        return 0U;
    }
    Afe_Delay_Ms(1U);

    if(0U == Gd30_Configure_Defaults()) {
        return 0U;
    }

    g_afe_gd30_config_fail_stage = GD30_CFG_FAIL_PROBE;
    if(0U == Gd30_Probe_Ready()) {
        return 0U;
    }

    g_afe_gd30_config_fail_stage = GD30_CFG_FAIL_NONE;
    return 1U;
}

static uint8_t Gd30_Read_Cells(bms_afe_data_t *data)
{
    uint16_t cell_mv;
    uint32_t i;

    if(data == 0) {
        return 0U;
    }

    for(i = 0U; i < BMS_CELL_COUNT; i++) {
        if(0U == Gd30_Direct_Read_U16(s_cell_voltage_regs[i], &cell_mv)) {
            return 0U;
        }
        data->cellMv[i] = cell_mv;
    }

    if(0U != Gd30_Direct_Read_U16(GD30BM2016_DIR_CELL9_VOLTAGE, &cell_mv)) {
        g_afe_gd30_raw_cell9_mv = cell_mv;
    }
    if(0U != Gd30_Direct_Read_U16(GD30BM2016_DIR_CELL16_VOLTAGE, &cell_mv)) {
        g_afe_gd30_raw_cell16_mv = cell_mv;
    }

    return 1U;
}

static void Gd30_Update_Top_Cell_Estimate(const bms_afe_data_t *data)
{
    uint32_t i;
    uint32_t low_sum_mv;

    if(data == 0) {
        return;
    }

    low_sum_mv = 0U;
    for(i = 0U; i < (BMS_CELL_COUNT - 1U); i++) {
        low_sum_mv += data->cellMv[i];
    }

    if(data->packVoltageMv > low_sum_mv) {
        g_afe_gd30_stack_minus_cell1_8_mv = (uint16_t)(data->packVoltageMv - low_sum_mv);
    } else {
        g_afe_gd30_stack_minus_cell1_8_mv = 0U;
    }
}

static void Afe_Update_Min_Max(bms_afe_data_t *data)
{
    uint32_t i;
    uint32_t sum_mv;
    uint16_t max_mv;
    uint16_t min_mv;

    max_mv = data->cellMv[0];
    min_mv = data->cellMv[0];
    sum_mv = 0U;

    for(i = 0U; i < BMS_CELL_COUNT; i++) {
        if(data->cellMv[i] > max_mv) {
            max_mv = data->cellMv[i];
        }
        if(data->cellMv[i] < min_mv) {
            min_mv = data->cellMv[i];
        }
        sum_mv += data->cellMv[i];
    }

    data->cellMaxMv = max_mv;
    data->cellMinMv = min_mv;
    data->cellDeltaMv = (uint16_t)(max_mv - min_mv);
    if(data->packVoltageMv == 0U) {
        data->packVoltageMv = (uint16_t)sum_mv;
    }
}

static uint8_t Gd30_Read_Stack_Voltage(bms_afe_data_t *data)
{
    uint16_t raw_mv;

    if(data == 0) {
        return 0U;
    }
    if(0U == Gd30_Direct_Read_U16(GD30BM2016_DIR_STACK_VOLTAGE, &raw_mv)) {
        return 0U;
    }

    data->packVoltageMv = (uint16_t)(raw_mv * GD30BM2016_STACK_PACK_LD_LSB_MV);
    return 1U;
}

static uint8_t Gd30_Read_Battery_Current(bms_afe_data_t *data)
{
    int16_t current_ma;

    if(data == 0) {
        return 0U;
    }
    if(0U == Gd30_Direct_Read_I16(GD30BM2016_DIR_CC2_CURRENT, &current_ma)) {
        return 0U;
    }

    data->batteryCurrentMa = current_ma;
    return 1U;
}

static uint8_t Gd30_Read_Temperatures(bms_afe_data_t *data)
{
    uint32_t i;
    int16_t raw_c;

    if(data == 0) {
        return 0U;
    }

    for(i = 0U; i < BMS_AFE_TEMP_COUNT; i++) {
        if(0U == Gd30_Direct_Read_I16(s_temperature_regs[i], &raw_c)) {
            return 0U;
        }
        /* BM2016 direct temperature commands already report in 0.1 degC. */
        data->temperaturesX10[i] = Gd30_Sanitize_Temperature_X10(raw_c);
    }

    return 1U;
}

static uint8_t Gd30_Read_Fault_Bitmap(uint32_t *fault_bitmap)
{
    uint8_t status[7];
#if BMS_ENABLE_AFE_FUSE_FLAG_MONITOR
    uint8_t fuse_flag;
#endif
    uint8_t ok;

    if(fault_bitmap == 0) {
        return 0U;
    }

    memset(status, 0, sizeof(status));
    ok = 1U;
    ok &= Gd30_Direct_Read(GD30BM2016_DIR_SAFETY_STATUS_A, &status[0], 1U);
    ok &= Gd30_Direct_Read(GD30BM2016_DIR_SAFETY_STATUS_B, &status[1], 1U);
    ok &= Gd30_Direct_Read(GD30BM2016_DIR_SAFETY_STATUS_C, &status[2], 1U);
#if BMS_ENABLE_AFE_PERMANENT_FAULT_MONITOR
    ok &= Gd30_Direct_Read(GD30BM2016_DIR_PF_STATUS_A, &status[3], 1U);
    ok &= Gd30_Direct_Read(GD30BM2016_DIR_PF_STATUS_B, &status[4], 1U);
    ok &= Gd30_Direct_Read(GD30BM2016_DIR_PF_STATUS_C, &status[5], 1U);
    ok &= Gd30_Direct_Read(GD30BM2016_DIR_PF_STATUS_D, &status[6], 1U);
#endif

    if(ok == 0U) {
        return 0U;
    }

    if((status[0] & GD30BM2016_SAFETY_A_COV) != 0U) {
        *fault_bitmap |= BMS_FAULT_CELL_OVP;
    }
    if((status[0] & GD30BM2016_SAFETY_A_CUV) != 0U) {
        *fault_bitmap |= BMS_FAULT_CELL_UVP;
    }
    if((status[0] & (GD30BM2016_SAFETY_A_OCC |
                     GD30BM2016_SAFETY_A_OCD1 |
                     GD30BM2016_SAFETY_A_OCD2)) != 0U) {
        *fault_bitmap |= BMS_FAULT_CHARGE_OCP;
    }
    if((status[0] & GD30BM2016_SAFETY_A_SCD) != 0U) {
        *fault_bitmap |= BMS_FAULT_SHORT_CIRCUIT;
    }
    if((status[1] & GD30BM2016_SAFETY_B_OVERTEMP_MASK) != 0U) {
        *fault_bitmap |= BMS_FAULT_OTP;
    }
    if((status[2] & GD30BM2016_SAFETY_C_COVL) != 0U) {
        *fault_bitmap |= BMS_FAULT_CELL_OVP;
    }
    if((status[2] & (GD30BM2016_SAFETY_C_OCD3 |
                     GD30BM2016_SAFETY_C_OCDL)) != 0U) {
        *fault_bitmap |= BMS_FAULT_CHARGE_OCP;
    }
    if((status[2] & GD30BM2016_SAFETY_C_SCDL) != 0U) {
        *fault_bitmap |= BMS_FAULT_SHORT_CIRCUIT;
    }
    if((status[0] | status[1] | status[2]) != 0U) {
        *fault_bitmap |= BMS_FAULT_AFE_PROTECTION;
    }
#if BMS_ENABLE_AFE_PERMANENT_FAULT_MONITOR
    if((status[3] | status[4] | status[5] | status[6]) != 0U) {
        *fault_bitmap |= BMS_FAULT_AFE_PROTECTION;
    }
#endif

#if BMS_ENABLE_AFE_FUSE_FLAG_MONITOR
    fuse_flag = 0U;
    if(0U != Gd30_Data_Memory_Read(GD30BM2016_CMD_FUSE_FLAG, &fuse_flag, 1U)) {
        if(fuse_flag != 0U) {
            *fault_bitmap |= BMS_FAULT_FUSE;
        }
    }
#endif

    return 1U;
}

static uint8_t Gd30_Read_Manufacturing_Status(uint8_t *manufacturing_status)
{
    if(manufacturing_status == 0) {
        return 0U;
    }

    *manufacturing_status = 0U;
    return Gd30_Data_Memory_Read(GD30BM2016_CMD_MANUFACTURING_STATUS,
                                 manufacturing_status,
                                 1U);
}

static void Gd30_Prepare_Fet_Command_Mode(void)
{
    uint16_t battery_status;

    battery_status = 0U;
    if(0U != Gd30_Read_Battery_Status(&battery_status)) {
        g_afe_gd30_last_battery_status = battery_status;
        if((battery_status & 0x0001U) != 0U) {
            (void)Gd30_Exit_Config_Update();
            Afe_Delay_Ms(5U);
        }
    }

    (void)Gd30_Subcommand_Only(GD30BM2016_SUBCMD_SLEEP_DISABLE);
    Afe_Delay_Ms(2U);
}

static uint8_t Gd30_Set_Normal_Fet_Control(uint8_t normal_enable)
{
    uint8_t manufacturing_status;
    uint8_t retry;
    uint8_t wanted;

    wanted = (normal_enable != 0U) ? GD30BM2016_MANUFACTURING_STATUS_FET_EN : 0U;
    if(0U == Gd30_Read_Manufacturing_Status(&manufacturing_status)) {
        return 0U;
    }
    if((manufacturing_status & GD30BM2016_MANUFACTURING_STATUS_FET_EN) == wanted) {
        g_afe_gd30_config_fail_stage = GD30_CFG_FAIL_NONE;
        g_afe_gd30_config_fail_step = GD30_CFG_FAIL_STEP_NONE;
        return 1U;
    }

    g_afe_gd30_config_fail_stage = GD30_CFG_FAIL_FET_ENABLE;
    Gd30_Prepare_Fet_Command_Mode();

    /* FET_ENABLE is a toggle, so always read back before issuing it again. */
    for(retry = 0U; retry < GD30BM2016_FET_ENABLE_RETRY_COUNT; retry++) {
        if(0U == Gd30_Read_Manufacturing_Status(&manufacturing_status)) {
            return 0U;
        }
        if((manufacturing_status & GD30BM2016_MANUFACTURING_STATUS_FET_EN) == wanted) {
            g_afe_gd30_config_fail_stage = GD30_CFG_FAIL_NONE;
            g_afe_gd30_config_fail_step = GD30_CFG_FAIL_STEP_NONE;
            return 1U;
        }

        if(0U == Gd30_Subcommand_Only(GD30BM2016_SUBCMD_FET_ENABLE)) {
            return 0U;
        }
        Afe_Delay_Ms(GD30BM2016_FET_ENABLE_RETRY_DELAY_MS);

        if(0U == Gd30_Read_Manufacturing_Status(&manufacturing_status)) {
            return 0U;
        }
        if((manufacturing_status & GD30BM2016_MANUFACTURING_STATUS_FET_EN) == wanted) {
            g_afe_gd30_config_fail_stage = GD30_CFG_FAIL_NONE;
            g_afe_gd30_config_fail_step = GD30_CFG_FAIL_STEP_NONE;
            return 1U;
        }
    }

    return 0U;
}

static uint8_t Gd30_Apply_Normal_Fets_Off(uint8_t *status)
{
    uint8_t ok;

    if(status != 0) {
        *status = 0U;
    }

    gpio_bit_set(BMS_AFE_DFETOFF_GPIO_PORT, BMS_AFE_DFETOFF_PIN);
    Afe_Delay_Ms(1U);

    ok = Gd30_Subcommand_Data_Write(GD30BM2016_CMD_FET_CONTROL,
                                    GD30BM2016_FET_CONTROL_ALL_OFF,
                                    1U);
    if(ok != 0U) {
        Afe_Delay_Ms(1U);
        (void)Gd30_Subcommand_Only(GD30BM2016_SUBCMD_CHG_PCHG_OFF);
        Afe_Delay_Ms(1U);
        (void)Gd30_Subcommand_Only(GD30BM2016_SUBCMD_DSG_PDSG_OFF);
        Afe_Delay_Ms(1U);
        (void)Gd30_Subcommand_Only(GD30BM2016_SUBCMD_ALL_FETS_OFF);
    }

    Afe_Delay_Ms(2U);

    if((ok != 0U) && (status != 0)) {
        ok = Gd30_Direct_Read(GD30BM2016_DIR_FET_STATUS, status, 1U);
    }

    return ok;
}

static uint8_t Gd30_Apply_Normal_Fets_On( uint8_t *status)
{
    uint8_t ok;
    uint8_t retry;

    if(status != 0) {
        *status = 0U;
    }

    gpio_bit_reset(BMS_AFE_DFETOFF_GPIO_PORT, BMS_AFE_DFETOFF_PIN);
    Afe_Delay_Ms(2U);

    ok = Gd30_Set_Normal_Fet_Control(1U);
    if(ok != 0U) {
        ok = Gd30_Subcommand_Only(GD30BM2016_SUBCMD_ALL_FETS_ON);
    }
    if(ok != 0U) {
        Afe_Delay_Ms(10U);
        ok = Gd30_Subcommand_Data_Write(GD30BM2016_CMD_FET_CONTROL, 0U, 1U);
    }

    Afe_Delay_Ms(10U);

    if((ok != 0U) && (status != 0)) {
        for(retry = 0U; retry < 5U; retry++) {
            ok = Gd30_Direct_Read(GD30BM2016_DIR_FET_STATUS, status, 1U);
            if((ok == 0U) ||
               ((*status & GD30BM2016_MAIN_PATH_FET_MASK) == GD30BM2016_MAIN_PATH_FET_MASK)) {
                break;
            }
            Afe_Delay_Ms(10U);
        }
    }

    return ok;
}

static uint8_t Gd30_Fet_Control_Off_Bits_From_Mask(uint8_t fet_mask)
{
    uint8_t off_bits;

    off_bits = 0U;
    if((fet_mask & AFE_GD30BM2016_FET_STATUS_DSG) == 0U) {
        off_bits |= GD30BM2016_FET_CONTROL_DSG_OFF;
    }
    if((fet_mask & AFE_GD30BM2016_FET_STATUS_PDSG) == 0U) {
        off_bits |= GD30BM2016_FET_CONTROL_PDSG_OFF;
    }
    if((fet_mask & AFE_GD30BM2016_FET_STATUS_CHG) == 0U) {
        off_bits |= GD30BM2016_FET_CONTROL_CHG_OFF;
    }
    if((fet_mask & AFE_GD30BM2016_FET_STATUS_PCHG) == 0U) {
        off_bits |= GD30BM2016_FET_CONTROL_PCHG_OFF;
    }

    return off_bits;
}

static uint8_t Afe_Read_Sideband_Status(void)
{
    uint8_t status;

    status = 0U;
    if(RESET != gpio_input_bit_get(BMS_AFE_DDSG_DCHG_GPIO_PORT, BMS_AFE_DCHG_PIN)) {
        status |= AFE_GD30BM2016_FET_STATUS_DCHG_PIN;
    }
    if(RESET != gpio_input_bit_get(BMS_AFE_DDSG_DCHG_GPIO_PORT, BMS_AFE_DDSG_PIN)) {
        status |= AFE_GD30BM2016_FET_STATUS_DDSG_PIN;
    }
    if(RESET == gpio_input_bit_get(BMS_AFE_ALERT_GPIO_PORT, BMS_AFE_ALERT_PIN)) {
        status |= AFE_GD30BM2016_FET_STATUS_ALERT_PIN;
    }
    if(RESET != gpio_output_bit_get(BMS_AFE_DFETOFF_GPIO_PORT, BMS_AFE_DFETOFF_PIN)) {
        status |= AFE_GD30BM2016_FET_STATUS_DFETOFF_HIGH;
    }

    return status;
}

static uint8_t Gd30_Read_Fet_Status_Combined(uint8_t *combined_status, uint8_t *raw_status)
{
    uint8_t raw;

    if(combined_status == 0) {
        return 0U;
    }

    *combined_status = Afe_Read_Sideband_Status();
    if(raw_status != 0) {
        *raw_status = 0U;
    }

    if(0U == Gd30_Direct_Read(GD30BM2016_DIR_FET_STATUS, &raw, 1U)) {
        return 0U;
    }

    if(raw_status != 0) {
        *raw_status = raw;
    }
    *combined_status = (uint8_t)((raw & GD30BM2016_FET_STATUS_OUTPUT_MASK) |
                                 Afe_Read_Sideband_Status());
    return 1U;
}

static uint8_t Gd30_Apply_Normal_Fet_Mask(uint8_t fet_mask, uint8_t *status)
{
    uint8_t ok;
    uint8_t off_bits;

    if(status != 0) {
        *status = 0U;
    }
    fet_mask &= GD30BM2016_FET_STATUS_OUTPUT_MASK;

    if(fet_mask != 0U) {
        gpio_bit_reset(BMS_AFE_DFETOFF_GPIO_PORT, BMS_AFE_DFETOFF_PIN);
        Afe_Delay_Ms(2U);

        ok = Gd30_Set_Normal_Fet_Control(1U);
        if(ok != 0U) {
            ok = Gd30_Subcommand_Only(GD30BM2016_SUBCMD_ALL_FETS_ON);
        }
        if(ok != 0U) {
            Afe_Delay_Ms(10U);
            off_bits = Gd30_Fet_Control_Off_Bits_From_Mask(fet_mask);
            ok = Gd30_Subcommand_Data_Write(GD30BM2016_CMD_FET_CONTROL, off_bits, 1U);
        }
    } else {
        gpio_bit_set(BMS_AFE_DFETOFF_GPIO_PORT, BMS_AFE_DFETOFF_PIN);
        Afe_Delay_Ms(2U);
        ok = Gd30_Subcommand_Data_Write(GD30BM2016_CMD_FET_CONTROL,
                                        GD30BM2016_FET_CONTROL_ALL_OFF,
                                        1U);
        if(ok != 0U) {
            Afe_Delay_Ms(1U);
            (void)Gd30_Subcommand_Only(GD30BM2016_SUBCMD_CHG_PCHG_OFF);
            Afe_Delay_Ms(1U);
            (void)Gd30_Subcommand_Only(GD30BM2016_SUBCMD_DSG_PDSG_OFF);
            Afe_Delay_Ms(1U);
            (void)Gd30_Subcommand_Only(GD30BM2016_SUBCMD_ALL_FETS_OFF);
        }
    }

    Afe_Delay_Ms(10U);
    if((ok != 0U) && (status != 0)) {
        ok = Gd30_Read_Fet_Status_Combined(status, 0);
        if(ok != 0U) {
            if(fet_mask != 0U) {
                ok = ((*status & fet_mask) == fet_mask) ? 1U : 0U;
            } else {
                ok = ((*status & GD30BM2016_FET_STATUS_OUTPUT_MASK) == 0U) ? 1U : 0U;
            }
        }
    }

    return ok;
}

static uint8_t Gd30_Apply_Preconnect_Test_Fet_Mask(uint8_t fet_mask, uint8_t *status)
{
    uint8_t ok;

    if(status != 0) {
        *status = 0U;
    }
    fet_mask &= GD30BM2016_PRECONNECT_FET_MASK;

    gpio_bit_reset(BMS_AFE_DFETOFF_GPIO_PORT, BMS_AFE_DFETOFF_PIN);
    Afe_Delay_Ms(2U);

    ok = Gd30_Set_Normal_Fet_Control(0U);
    if(ok != 0U) {
        ok = Gd30_Subcommand_Data_Write(GD30BM2016_CMD_FET_CONTROL, 0U, 1U);
    }
    if(ok != 0U) {
        Afe_Delay_Ms(1U);
        ok = Gd30_Subcommand_Only(GD30BM2016_SUBCMD_ALL_FETS_OFF);
    }
    if((ok != 0U) &&
       ((fet_mask & AFE_GD30BM2016_FET_STATUS_PCHG) != 0U)) {
        Afe_Delay_Ms(1U);
        ok = Gd30_Subcommand_Only(GD30BM2016_SUBCMD_PCHGTEST);
    }
    if((ok != 0U) &&
       ((fet_mask & AFE_GD30BM2016_FET_STATUS_PDSG) != 0U)) {
        Afe_Delay_Ms(1U);
        ok = Gd30_Subcommand_Only(GD30BM2016_SUBCMD_PDSGTEST);
    }

    Afe_Delay_Ms(10U);
    if((ok != 0U) && (status != 0)) {
        ok = Gd30_Read_Fet_Status_Combined(status, 0);
        if(ok != 0U) {
            ok = ((*status & fet_mask) == fet_mask) ? 1U : 0U;
        }
    }

    return ok;
}

static uint8_t Gd30_Read_Snapshot(bms_afe_data_t *data, uint32_t *fault_bitmap)
{
    if(0U == Gd30_I2c_Lock(portMAX_DELAY)) {
        return 0U;
    }

    if(0U == Gd30_Read_Cells(data)) {
        s_i2c_ready = 0U;
        Gd30_I2c_Unlock();
        return 0U;
    }

    (void)Gd30_Read_Stack_Voltage(data);
    if(0U == Gd30_Read_Battery_Current(data)) {
        s_i2c_ready = 0U;
        Gd30_I2c_Unlock();
        return 0U;
    }
    Gd30_Update_Top_Cell_Estimate(data);
    (void)Gd30_Read_Temperatures(data);
    (void)Gd30_Read_Fault_Bitmap(fault_bitmap);
    Afe_Update_Min_Max(data);
    Gd30_I2c_Unlock();

    return 1U;
}

static uint8_t Gd30_Recover_If_Due(void)
{
    TickType_t now;

    if(xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return 0U;
    }

    now = xTaskGetTickCount();
    if((TickType_t)(now - s_last_recover_tick) < pdMS_TO_TICKS(AFE_RECOVER_PERIOD_MS)) {
        return 0U;
    }
    s_last_recover_tick = now;

    if(0U == Gd30_I2c_Lock(0U)) {
        return 0U;
    }

    s_i2c_ready = Gd30_Start_I2c_Link();
    Gd30_I2c_Unlock();

    return s_i2c_ready;
}

void Afe_Gd30bm2016_Init(void)
{
    s_balance_bitmap = 0U;
    s_i2c_addr_write = GD30BM2016_I2C_ADDR_WRITE;
    if(s_i2c_mutex == 0) {
        s_i2c_mutex = xSemaphoreCreateMutex();
        configASSERT(s_i2c_mutex != 0);
    }

    s_i2c_ready = 0U;
    s_last_recover_tick = (TickType_t)(0U - pdMS_TO_TICKS(AFE_RECOVER_PERIOD_MS));
    s_i2c_ready = Gd30_Start_I2c_Link();
}

void Afe_Gd30bm2016_Poll(bms_afe_data_t *data)
{
    uint32_t fault_bitmap;

    if(data == 0) {
        return;
    }

    fault_bitmap = 0U;
    memset(data, 0, sizeof(*data));

    if(s_i2c_ready != 0U) {
        if(0U != Gd30_Read_Snapshot(data, &fault_bitmap)) {
            data->faultBitmap = fault_bitmap;
            return;
        }
    }

    if(0U != Gd30_Recover_If_Due()) {
        if(0U != Gd30_Read_Snapshot(data, &fault_bitmap)) {
            data->faultBitmap = fault_bitmap;
            return;
        }
    }

    data->faultBitmap = BMS_FAULT_AFE_COMM;
}

void Afe_Gd30bm2016_Set_Balance(uint16_t balance_bitmap)
{
#if GD30BM2016_BALANCE_REGISTER_VALID
    uint16_t hw_balance_bitmap;
#endif

    s_balance_bitmap = (uint16_t)(balance_bitmap & ((1U << BMS_CELL_COUNT) - 1U));

#if GD30BM2016_BALANCE_REGISTER_VALID
    hw_balance_bitmap = Gd30_Balance_Logical_To_Hw(s_balance_bitmap);
    if(s_i2c_ready != 0U) {
        if(0U != Gd30_I2c_Lock(portMAX_DELAY)) {
            (void)Gd30_Subcommand_Data_Write(GD30BM2016_CMD_CELL_BALANCING_ACTIVE_CELLS,
                                             hw_balance_bitmap,
                                             2U);
            Gd30_I2c_Unlock();
        }
    }
#endif
}

void Afe_Gd30bm2016_Force_Path_Off_Fast(void)
{
    gpio_bit_set(BMS_AFE_DFETOFF_GPIO_PORT, BMS_AFE_DFETOFF_PIN);
}

void Afe_Gd30bm2016_Fets_Off(void)
{
    Afe_Gd30bm2016_Force_Path_Off_Fast();

    if(s_i2c_ready != 0U) {
        if(0U != Gd30_I2c_Lock(portMAX_DELAY)) {
            (void)Gd30_Apply_Normal_Fets_Off(0);
            Gd30_I2c_Unlock();
        }
    }
}

uint8_t Afe_Gd30bm2016_Fets_On(uint8_t *status)
{
    uint8_t fet_status;
    uint8_t ok;

    if(status != 0) {
        *status = 0U;
    }
    if(s_i2c_ready != 0U) {
        if(0U != Gd30_I2c_Lock(portMAX_DELAY)) {
            (void)Gd30_Subcommand_Only(GD30BM2016_SUBCMD_OCDL_RECOVER);
            Afe_Delay_Ms(2U);
            (void)Gd30_Subcommand_Only(GD30BM2016_SUBCMD_SCDL_RECOVER);
            Afe_Delay_Ms(2U);
            ok = Gd30_Apply_Normal_Fets_On(&fet_status);
            Gd30_I2c_Unlock();
            if(status != 0) {
                *status = fet_status;
            }
            return ((ok != 0U) &&
                    ((fet_status & GD30BM2016_MAIN_PATH_FET_MASK) ==
                     GD30BM2016_MAIN_PATH_FET_MASK)) ? 1U : 0U;
        }
    }

    return 0U;
}

uint8_t Afe_Gd30bm2016_Recover_Protections(void)
{
    uint8_t ok;

    if(s_i2c_ready == 0U) {
        return 0U;
    }
    if(0U == Gd30_I2c_Lock(portMAX_DELAY)) {
        return 0U;
    }

    ok = 1U;
    ok &= Gd30_Subcommand_Only(GD30BM2016_SUBCMD_OCDL_RECOVER);
    Afe_Delay_Ms(2U);
    ok &= Gd30_Subcommand_Only(GD30BM2016_SUBCMD_SCDL_RECOVER);
    Afe_Delay_Ms(2U);
    Gd30_I2c_Unlock();

    return ok;
}

uint8_t Afe_Gd30bm2016_Set_Fet_Mask(uint8_t fet_mask, uint8_t *status)
{
    uint8_t ok;
    uint8_t requested_mask;

    if(s_i2c_ready == 0U) {
        if(status != 0) {
            *status = 0U;
        }
        return 0U;
    }
    if(0U == Gd30_I2c_Lock(portMAX_DELAY)) {
        if(status != 0) {
            *status = 0U;
        }
        return 0U;
    }

    requested_mask = (uint8_t)(fet_mask & GD30BM2016_FET_STATUS_OUTPUT_MASK);
    if(requested_mask != 0U) {
        (void)Gd30_Subcommand_Only(GD30BM2016_SUBCMD_OCDL_RECOVER);
        Afe_Delay_Ms(2U);
        (void)Gd30_Subcommand_Only(GD30BM2016_SUBCMD_SCDL_RECOVER);
        Afe_Delay_Ms(2U);
    }
    if((requested_mask != 0U) &&
       ((requested_mask & (uint8_t)~GD30BM2016_PRECONNECT_FET_MASK) == 0U)) {
        ok = Gd30_Apply_Preconnect_Test_Fet_Mask(requested_mask, status);
    } else {
        ok = Gd30_Apply_Normal_Fet_Mask(requested_mask, status);
    }
    Gd30_I2c_Unlock();
    return ok;
}

uint8_t Afe_Gd30bm2016_Read_Fet_Status(uint8_t *status)
{
    if(status == 0) {
        return 0U;
    }

    *status = 0U;
    if(s_i2c_ready == 0U) {
        return 0U;
    }
    if(0U == Gd30_I2c_Lock(portMAX_DELAY)) {
        return 0U;
    }
    if(0U == Gd30_Read_Fet_Status_Combined(status, 0)) {
        Gd30_I2c_Unlock();
        return 0U;
    }

    Gd30_I2c_Unlock();
    return 1U;
}

void Afe_Gd30bm2016_Get_Debug(afe_gd30bm2016_debug_t *debug)
{
    uint8_t saved_fail_step;
    uint16_t saved_fail_reg;

    if(debug == 0) {
        return;
    }

    memset(debug, 0, sizeof(*debug));
    debug->configFailStage = g_afe_gd30_config_fail_stage;
    debug->configFailStep = g_afe_gd30_config_fail_step;
    debug->configFailIndex = g_afe_gd30_config_fail_index;
    debug->cfgupdateSeen = g_afe_gd30_cfgupdate_seen;
    debug->i2cAddrWrite = g_afe_gd30_i2c_addr_write;
    debug->lastReg = g_afe_gd30_i2c_last_reg;
    debug->lastCrcOk = g_afe_gd30_i2c_last_crc_ok;
    debug->lastCrcRx = g_afe_gd30_i2c_last_crc_rx;
    debug->lastCrcCalc = g_afe_gd30_i2c_last_crc_calc;
    debug->configFailReg = g_afe_gd30_config_fail_reg;
    debug->lastBatteryStatus = g_afe_gd30_last_battery_status;
    debug->probeVcellMode = g_afe_gd30_probe_vcell_mode;
    debug->rawCell9Mv = g_afe_gd30_raw_cell9_mv;
    debug->rawCell16Mv = g_afe_gd30_raw_cell16_mv;
    debug->stackMinusCell1_8Mv = g_afe_gd30_stack_minus_cell1_8_mv;

    saved_fail_step = g_afe_gd30_config_fail_step;
    saved_fail_reg = g_afe_gd30_config_fail_reg;
    if((s_i2c_ready != 0U) && (0U != Gd30_I2c_Lock(portMAX_DELAY))) {
        (void)Gd30_Direct_Read(GD30BM2016_DIR_SAFETY_STATUS_A, &debug->safetyStatusA, 1U);
        (void)Gd30_Direct_Read(GD30BM2016_DIR_SAFETY_STATUS_B, &debug->safetyStatusB, 1U);
        (void)Gd30_Direct_Read(GD30BM2016_DIR_SAFETY_STATUS_C, &debug->safetyStatusC, 1U);
        (void)Gd30_Read_Manufacturing_Status(&debug->manufacturingStatus);
        (void)Gd30_Data_Memory_Read(GD30BM2016_DM_FET_OPTIONS, &debug->fetOptions, 1U);
        Gd30_I2c_Unlock();
        g_afe_gd30_config_fail_step = saved_fail_step;
        g_afe_gd30_config_fail_reg = saved_fail_reg;
    }
}

uint8_t Afe_Gd30bm2016_Read_Path_Voltages(uint16_t *stack_mv, uint16_t *pack_mv, uint16_t *ld_mv)
{
    uint16_t raw_stack;
    uint16_t raw_pack;
    uint16_t raw_ld;
    uint8_t ok;

    if((stack_mv == 0) || (pack_mv == 0) || (ld_mv == 0)) {
        return 0U;
    }

    *stack_mv = 0U;
    *pack_mv = 0U;
    *ld_mv = 0U;

    if(s_i2c_ready == 0U) {
        return 0U;
    }
    if(0U == Gd30_I2c_Lock(portMAX_DELAY)) {
        return 0U;
    }

    ok = 1U;
    ok &= Gd30_Direct_Read_U16(GD30BM2016_DIR_STACK_VOLTAGE, &raw_stack);
    ok &= Gd30_Direct_Read_U16(GD30BM2016_DIR_PACK_PIN_VOLTAGE, &raw_pack);
    ok &= Gd30_Direct_Read_U16(GD30BM2016_DIR_LD_PIN_VOLTAGE, &raw_ld);
    if(ok != 0U) {
        *stack_mv = (uint16_t)(raw_stack * GD30BM2016_STACK_PACK_LD_LSB_MV);
        *pack_mv = (uint16_t)(raw_pack * GD30BM2016_STACK_PACK_LD_LSB_MV);
        *ld_mv = (uint16_t)(raw_ld * GD30BM2016_STACK_PACK_LD_LSB_MV);
    }

    Gd30_I2c_Unlock();
    return ok;
}

uint8_t Afe_Gd30bm2016_Alert_Active(void)
{
    return (RESET == gpio_input_bit_get(BMS_AFE_ALERT_GPIO_PORT, BMS_AFE_ALERT_PIN)) ? 1U : 0U;
}
