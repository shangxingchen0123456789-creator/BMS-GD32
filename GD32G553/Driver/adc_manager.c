#include "adc_manager.h"

#include "bms_board_config.h"
#include "power_control.h"

#include <string.h>

#define ADC_USE_HARDWARE                       BMS_ENABLE_POWER_ADC
#define ADC_TEMP_RAW_MIN_VALID                 16U
#define ADC_TEMP_RAW_MAX_VALID                 (BMS_ADC_MAX_RAW - 16U)
#define ADC_VIN_PRESENT_MIN_MV                BMS_POWER_EXTERNAL_PRESENT_MV
#define ADC_VIN_PRESENT_MAX_MV                BMS_POWER_INPUT_PRESENT_MAX_MV
#define ADC_VIN_RAW_MIN_VALID                 16U
#define ADC_VIN_RAW_MAX_VALID                 (BMS_ADC_MAX_RAW - 16U)
#define ADC_VOLTAGE_AVERAGE_SAMPLES           8U
#define ADC_CURRENT_AVERAGE_SAMPLES           32U
#define ADC_TEMP_AVERAGE_SAMPLES              8U
#define ADC_VOLTAGE_IIR_SHIFT                 2U
#define ADC_CURRENT_IIR_SHIFT                 3U
#define ADC_TEMP_IIR_SHIFT                    3U

typedef struct {
    uint16_t raw;
    int16_t tempX10;
} adc_ntc_point_t;

static const adc_ntc_point_t s_adc_ntc_table[] = {
    {  99U,  -400},
    { 140U,  -350},
    { 195U,  -300},
    { 265U,  -250},
    { 355U,  -200},
    { 466U,  -150},
    { 600U,  -100},
    { 758U,   -50},
    { 939U,     0},
    {1140U,    50},
    {1357U,   100},
    {1585U,   150},
    {1817U,   200},
    {2048U,   250},
    {2270U,   300},
    {2481U,   350},
    {2676U,   400},
    {2854U,   450},
    {3014U,   500},
    {3155U,   550},
    {3280U,   600},
    {3388U,   650},
    {3482U,   700},
    {3563U,   750},
    {3633U,   800},
    {3694U,   850},
    {3745U,   900},
    {3790U,   950},
    {3828U,  1000},
    {3861U,  1050},
    {3889U,  1100},
    {3914U,  1150},
    {3935U,  1200},
    {3953U,  1250}
};

/*
 * 功率板真实采样管理。
 *
 * 本模块只做一件事：把 MCU ADC 读取到的真实硬件原始值换算成工程内部
 * 使用的物理量。GD32G553_power 测试工程已经验证 Buck、Boost 和 ADC 采样
 * 均正常，因此主工程不再保留任何软件造数兜底。
 *
 * 设计规则：
 * - ADC 成功：输出 Vin/Vout/Iin/Iout/MOS 温度/电感温度的真实采样；
 * - ADC 未启用、未初始化或任一路转换失败：输出结构保持 0，并置 BMS_FAULT_ADC；
 * - 上层状态机看到 BMS_FAULT_ADC 后会进入故障保护，不会把 0 值当作正常数据。
 */
#if ADC_USE_HARDWARE
static uint8_t s_adc_ready;
static uint32_t s_adc_fault;
static bms_power_sample_t s_filtered_sample;
static uint8_t s_filter_valid;

static void Adc_Short_Delay(void)
{
    volatile uint32_t i;

    /*
     * ADC enable / calibration 后需要等待内部模拟电路稳定。
     * 这里不用 RTOS 延时，避免底层初始化依赖调度器。
     */
    for(i = 0U; i < 8000U; i++) {
    }
}

static uint16_t Adc_Raw_To_Mv(uint16_t raw)
{
    /* 12 位 ADC 原始值换算为引脚电压，参考电压来自 bms_board_config.h。 */
    return (uint16_t)(((uint32_t)raw * BMS_ADC_VREF_MV) / BMS_ADC_MAX_RAW);
}

static uint16_t Adc_Scale_Voltage_Mv(uint16_t raw, uint32_t gain_x1000)
{
    uint32_t mv;

    /*
     * gain_x1000 是分压/差分采样链路的反向比例：
     * ADC 引脚电压 mV * gain_x1000 / 1000 = 被测端真实电压 mV。
     */
    mv = (uint32_t)Adc_Raw_To_Mv(raw);
    mv = (mv * gain_x1000) / 1000U;
    if(mv > 65535U) {
        mv = 65535U;
    }

    return (uint16_t)mv;
}

static int16_t Adc_Clamp_I16(int32_t value)
{
    if(value > 32767) {
        return 32767;
    }
    if(value < -32768) {
        return -32768;
    }

    return (int16_t)value;
}

static int16_t Adc_Scale_Current_Ma(uint16_t raw, uint16_t offset_raw, uint32_t ma_per_mv_x1000)
{
    int32_t diff_raw;
    int32_t mv;
    int32_t ma;

    /*
     * 电流换算沿用测试电源板已经验证的 5mR * 约 62 倍采样链：
     * 1A -> ADC 引脚约 0.31V，所以 1mV -> 3.226mA。
     * ma_per_mv_x1000 使用 x1000 保留小数精度，最后再除以 1000。
     */
    diff_raw = (int32_t)raw - (int32_t)offset_raw;
    mv = (diff_raw * (int32_t)BMS_ADC_VREF_MV) / (int32_t)BMS_ADC_MAX_RAW;
    ma = (mv * (int32_t)ma_per_mv_x1000) / 1000;

    return Adc_Clamp_I16(ma);
}

static int16_t Adc_Scale_Temp_X10(uint16_t raw)
{
    int32_t temp_x10;
    int32_t raw_low;
    int32_t raw_high;
    int32_t temp_low;
    int32_t temp_high;
    uint32_t i;
    const uint32_t count = sizeof(s_adc_ntc_table) / sizeof(s_adc_ntc_table[0]);

    /*
     * 功率板温度采样为 P3V3 -> 10k/B3950 NTC -> ADC -> 10k -> GND。
     * NTC 升温时阻值下降，ADC 原始码上升；表内按 5 摄氏度步进做线性插值。
     */
    if(raw <= s_adc_ntc_table[0].raw) {
        return s_adc_ntc_table[0].tempX10;
    }
    if(raw >= s_adc_ntc_table[count - 1U].raw) {
        return s_adc_ntc_table[count - 1U].tempX10;
    }

    temp_x10 = s_adc_ntc_table[count - 1U].tempX10;
    for(i = 1U; i < count; i++) {
        if(raw <= s_adc_ntc_table[i].raw) {
            raw_low = (int32_t)s_adc_ntc_table[i - 1U].raw;
            raw_high = (int32_t)s_adc_ntc_table[i].raw;
            temp_low = (int32_t)s_adc_ntc_table[i - 1U].tempX10;
            temp_high = (int32_t)s_adc_ntc_table[i].tempX10;
            temp_x10 = temp_low +
                       (((int32_t)raw - raw_low) * (temp_high - temp_low)) /
                       (raw_high - raw_low);
            break;
        }
    }

    if(temp_x10 > BMS_TEMP_MAX_VALID_X10) {
        temp_x10 = BMS_TEMP_MAX_VALID_X10;
    } else if(temp_x10 < BMS_TEMP_MIN_VALID_X10) {
        temp_x10 = BMS_TEMP_MIN_VALID_X10;
    }

    return (int16_t)temp_x10;
}

static uint8_t Adc_Temp_Raw_Valid(uint16_t raw)
{
    return (raw > ADC_TEMP_RAW_MIN_VALID && raw < ADC_TEMP_RAW_MAX_VALID) ? 1U : 0U;
}

static uint8_t Adc_Vin_Present(uint16_t vin_raw, uint16_t vin_mv)
{
    if(vin_raw <= ADC_VIN_RAW_MIN_VALID || vin_raw >= ADC_VIN_RAW_MAX_VALID) {
        return 0U;
    }
    if(vin_mv < ADC_VIN_PRESENT_MIN_MV || vin_mv > ADC_VIN_PRESENT_MAX_MV) {
        return 0U;
    }
#if BMS_POWER_FLOATING_VIN_REJECT_ENABLE
    /*
     * On the current power-board harness, an unconnected VIN input can float
     * around 16 V after scaling. Treat that board-specific window as absent
     * input so the charger does not start from a false ADC_VIN reading.
     */
    if(vin_mv >= BMS_POWER_FLOATING_VIN_MIN_MV &&
       vin_mv <= BMS_POWER_FLOATING_VIN_MAX_MV) {
        return 0U;
    }
#endif

    return 1U;
}

static void Adc_Sanitize_Absent_Power(bms_power_sample_t *sample, uint8_t vin_present)
{
    if(sample == 0) {
        return;
    }

    /*
     * ADC_Vin only decides whether a charger is connected to the power-board
     * input. Keep ADC_Vout and ADC_Iout visible because they describe the
     * power-board output side and help diagnose a Vin-present false negative.
     */
    if(vin_present == 0U) {
        sample->inputVoltageMv = 0U;
        sample->inputCurrentMa = 0;
        sample->faultOcActive = 0U;
    }
}

static void Adc_Gpio_Init(void)
{
    /*
     * ADC 引脚必须配置为模拟模式并关闭上下拉，避免数字输入缓冲或上下拉
     * 对电压分压、RC 滤波和运放输出造成额外负载。
     */
    rcu_periph_clock_enable(BMS_ADC_IIN_GPIO_CLK);
    rcu_periph_clock_enable(BMS_ADC_VIN_GPIO_CLK);
    rcu_periph_clock_enable(BMS_ADC_VOUT_GPIO_CLK);
    rcu_periph_clock_enable(BMS_ADC_IOUT_GPIO_CLK);
    rcu_periph_clock_enable(BMS_ADC_MOS_TEMP_GPIO_CLK);
    rcu_periph_clock_enable(BMS_ADC_L_TEMP_GPIO_CLK);

    gpio_mode_set(BMS_ADC_IIN_GPIO_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, BMS_ADC_IIN_PIN);
    gpio_mode_set(BMS_ADC_VIN_GPIO_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, BMS_ADC_VIN_PIN);
    gpio_mode_set(BMS_ADC_VOUT_GPIO_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, BMS_ADC_VOUT_PIN);
    gpio_mode_set(BMS_ADC_IOUT_GPIO_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, BMS_ADC_IOUT_PIN);
    gpio_mode_set(BMS_ADC_MOS_TEMP_GPIO_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, BMS_ADC_MOS_TEMP_PIN);
    gpio_mode_set(BMS_ADC_L_TEMP_GPIO_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, BMS_ADC_L_TEMP_PIN);
}

static void Adc_One_Init(uint32_t adc_periph)
{
    /*
     * 软件触发、单通道、非连续转换：
     * 每次 Adc_Read_Channel() 都显式选择一路 ADC 并等待 EOC，通道顺序清晰，
     * 方便和主控板/电源板网表逐项核对。
     */
    adc_deinit(adc_periph);
    adc_clock_config(adc_periph, ADC_CLK_SYNC_HCLK_DIV16);
    adc_special_function_config(adc_periph, ADC_SCAN_MODE, DISABLE);
    adc_special_function_config(adc_periph, ADC_CONTINUOUS_MODE, DISABLE);
    adc_data_alignment_config(adc_periph, ADC_DATAALIGN_RIGHT);
    adc_resolution_config(adc_periph, ADC_RESOLUTION_12B);
    adc_channel_length_config(adc_periph, ADC_ROUTINE_CHANNEL, 1U);
    adc_external_trigger_config(adc_periph, ADC_ROUTINE_CHANNEL, EXTERNAL_TRIGGER_DISABLE);
    adc_end_of_conversion_config(adc_periph, ADC_EOC_SET_CONVERSION);
    adc_calibration_mode_config(adc_periph, ADC_CALIBRATION_OFFSET_MISMATCH);
    adc_calibration_number(adc_periph, ADC_CALIBRATION_NUM16);
    adc_enable(adc_periph);
    Adc_Short_Delay();
    adc_calibration_enable(adc_periph);
}

static uint8_t Adc_Read_Channel_With_Sample_Time(uint32_t adc_periph,
                                                 uint8_t channel,
                                                 uint32_t sample_time,
                                                 uint16_t *raw)
{
    uint32_t timeout;

    if(raw == 0) {
        return 0U;
    }

    adc_routine_channel_config(adc_periph, 0U, channel, sample_time);
    adc_flag_clear(adc_periph, ADC_FLAG_EOC);
    adc_software_trigger_enable(adc_periph, ADC_ROUTINE_CHANNEL);

    timeout = BMS_ADC_TIMEOUT;
    while((RESET == adc_flag_get(adc_periph, ADC_FLAG_EOC)) && (timeout > 0U)) {
        timeout--;
    }

    if(timeout == 0U) {
        s_adc_fault |= BMS_FAULT_ADC;
        return 0U;
    }

    *raw = (uint16_t)(adc_routine_data_read(adc_periph) & 0x0FFFU);
    adc_flag_clear(adc_periph, ADC_FLAG_EOC);

    return 1U;
}

static uint8_t Adc_Read_Channel(uint32_t adc_periph, uint8_t channel, uint16_t *raw)
{
    return Adc_Read_Channel_With_Sample_Time(adc_periph, channel, BMS_ADC_SAMPLE_TIME, raw);
}

static uint8_t Adc_Read_Current_Channel(uint32_t adc_periph, uint8_t channel, uint16_t *raw)
{
#if BMS_ENABLE_PWM_SYNC_ADC
    if(0U != Power_Control_Wait_Adc_Sample_Point(BMS_ADC_SYNC_WAIT_TIMEOUT)) {
        return Adc_Read_Channel_With_Sample_Time(adc_periph,
                                                 channel,
                                                 BMS_ADC_CURRENT_SAMPLE_TIME,
                                                 raw);
    }
#endif

    return Adc_Read_Channel(adc_periph, channel, raw);
}

static uint8_t Adc_Read_Channel_Average_Internal(uint32_t adc_periph,
                                                 uint8_t channel,
                                                 uint8_t sample_count,
                                                 uint16_t *raw,
                                                 uint8_t sync_to_pwm)
{
    uint32_t sum;
    uint16_t sample;
    uint8_t i;

    if(raw == 0) {
        return 0U;
    }
    if(sample_count == 0U) {
        sample_count = 1U;
    }

    sum = 0U;
    for(i = 0U; i < sample_count; i++) {
        if(sync_to_pwm != 0U) {
            if(0U == Adc_Read_Current_Channel(adc_periph, channel, &sample)) {
                return 0U;
            }
        } else if(0U == Adc_Read_Channel(adc_periph, channel, &sample)) {
            return 0U;
        }
        sum += sample;
    }

    *raw = (uint16_t)((sum + ((uint32_t)sample_count / 2U)) / (uint32_t)sample_count);
    return 1U;
}

static uint8_t Adc_Read_Channel_Average(uint32_t adc_periph,
                                        uint8_t channel,
                                        uint8_t sample_count,
                                        uint16_t *raw)
{
    return Adc_Read_Channel_Average_Internal(adc_periph, channel, sample_count, raw, 0U);
}

static uint8_t Adc_Read_Current_Channel_Average(uint32_t adc_periph,
                                                uint8_t channel,
                                                uint8_t sample_count,
                                                uint16_t *raw)
{
    return Adc_Read_Channel_Average_Internal(adc_periph, channel, sample_count, raw, 1U);
}

static uint8_t Adc_Read_Output_Current_Channel_Average(uint32_t adc_periph,
                                                       uint8_t channel,
                                                       uint8_t sample_count,
                                                       uint16_t *raw)
{
    power_control_state_t power_state;

    /*
     * In Boost mode the output current pulse happens while the boost low-side
     * MOS is off. The old fixed 80% PWM sample point can fall inside the
     * low-side on-time at 30%~60% boost duty, which reads near-zero Iout and
     * makes the current loop keep increasing duty. Average asynchronously in
     * Boost so the feedback represents the cycle average better.
     */
    memset(&power_state, 0, sizeof(power_state));
    Power_Control_Get_State(&power_state);
    if((power_state.enabled != 0U) &&
       ((power_state.powerStageMode == (uint8_t)POWER_STAGE_MODE_BOOST) ||
        (power_state.asyncBoostRectifier != 0U))) {
        return Adc_Read_Channel_Average(adc_periph, channel, sample_count, raw);
    }

    return Adc_Read_Current_Channel_Average(adc_periph, channel, sample_count, raw);
}

static int16_t Adc_Iir_I16(int16_t previous, int16_t current, uint8_t shift)
{
    int32_t delta;

    if(shift == 0U) {
        return current;
    }

    delta = (int32_t)current - (int32_t)previous;
    return Adc_Clamp_I16((int32_t)previous + (delta >> shift));
}

static uint16_t Adc_Iir_U16(uint16_t previous, uint16_t current, uint8_t shift)
{
    int32_t delta;
    int32_t next;

    if(shift == 0U) {
        return current;
    }

    delta = (int32_t)current - (int32_t)previous;
    next = (int32_t)previous + (delta >> shift);
    if(next < 0) {
        next = 0;
    } else if(next > 65535) {
        next = 65535;
    }

    return (uint16_t)next;
}

static void Adc_Filter_Sample(bms_power_sample_t *sample)
{
    if(sample == 0) {
        return;
    }

    if(sample->inputVoltageMv == 0U || sample->faultBitmap != 0U) {
        memset(&s_filtered_sample, 0, sizeof(s_filtered_sample));
        s_filter_valid = 0U;
        return;
    }

    if(s_filter_valid == 0U) {
        s_filtered_sample = *sample;
        s_filter_valid = 1U;
    } else {
        s_filtered_sample.inputVoltageMv =
            Adc_Iir_U16(s_filtered_sample.inputVoltageMv, sample->inputVoltageMv, ADC_VOLTAGE_IIR_SHIFT);
        s_filtered_sample.outputVoltageMv =
            Adc_Iir_U16(s_filtered_sample.outputVoltageMv, sample->outputVoltageMv, ADC_VOLTAGE_IIR_SHIFT);
        s_filtered_sample.inputCurrentMa =
            Adc_Iir_I16(s_filtered_sample.inputCurrentMa, sample->inputCurrentMa, ADC_CURRENT_IIR_SHIFT);
        s_filtered_sample.outputCurrentMa =
            Adc_Iir_I16(s_filtered_sample.outputCurrentMa, sample->outputCurrentMa, ADC_CURRENT_IIR_SHIFT);
        s_filtered_sample.mosTempX10 =
            Adc_Iir_I16(s_filtered_sample.mosTempX10, sample->mosTempX10, ADC_TEMP_IIR_SHIFT);
        s_filtered_sample.inductorTempX10 =
            Adc_Iir_I16(s_filtered_sample.inductorTempX10, sample->inductorTempX10, ADC_TEMP_IIR_SHIFT);
        s_filtered_sample.faultOcActive = sample->faultOcActive;
        s_filtered_sample.iinRaw = sample->iinRaw;
        s_filtered_sample.vinRaw = sample->vinRaw;
        s_filtered_sample.voutRaw = sample->voutRaw;
        s_filtered_sample.ioutRaw = sample->ioutRaw;
        s_filtered_sample.mosTempRaw = sample->mosTempRaw;
        s_filtered_sample.inductorTempRaw = sample->inductorTempRaw;
        s_filtered_sample.faultBitmap = sample->faultBitmap;
    }

    *sample = s_filtered_sample;
}

static uint8_t Adc_Sample_Hardware_Counts(bms_power_sample_t *sample,
                                          uint8_t voltage_samples,
                                          uint8_t current_samples,
                                          uint8_t temp_samples)
{
    uint16_t iin_raw;
    uint16_t vin_raw;
    uint16_t vout_raw;
    uint16_t iout_raw;
    uint16_t mos_temp_raw;
    uint16_t l_temp_raw;
    uint8_t mos_temp_valid;
    uint8_t l_temp_valid;
    uint8_t vin_present;

    /*
     * 顺序按 H1 网表排列，便于现场拿示波器/万用表逐路核对：
     * IIN -> VIN -> VOUT -> IOUT -> MOS 温度 -> 电感温度。
     * 任一路失败，本轮采样整体失败，禁止混用部分旧数据。
     */
    s_adc_fault = 0U;

    /*
     * Iin is a pulsed input-side current. Use asynchronous averaging so the
     * reported value is closer to the DC supply average instead of one PWM
     * phase point. Iout switches to asynchronous averaging in Boost mode for
     * the same reason; a fixed phase can miss the output-current pulse.
     */
    if(0U == Adc_Read_Channel_Average(BMS_ADC_IIN_PERIPH,
                                      BMS_ADC_IIN_CHANNEL,
                                      current_samples,
                                      &iin_raw)) {
        return 0U;
    }
    if(0U == Adc_Read_Channel_Average(BMS_ADC_VIN_PERIPH,
                                      BMS_ADC_VIN_CHANNEL,
                                      voltage_samples,
                                      &vin_raw)) {
        return 0U;
    }
    if(0U == Adc_Read_Channel_Average(BMS_ADC_VOUT_PERIPH,
                                      BMS_ADC_VOUT_CHANNEL,
                                      voltage_samples,
                                      &vout_raw)) {
        return 0U;
    }
    if(0U == Adc_Read_Output_Current_Channel_Average(BMS_ADC_IOUT_PERIPH,
                                                     BMS_ADC_IOUT_CHANNEL,
                                                     current_samples,
                                                     &iout_raw)) {
        return 0U;
    }
    if(0U == Adc_Read_Channel_Average(BMS_ADC_MOS_TEMP_PERIPH,
                                      BMS_ADC_MOS_TEMP_CHANNEL,
                                      temp_samples,
                                      &mos_temp_raw)) {
        return 0U;
    }
    if(0U == Adc_Read_Channel_Average(BMS_ADC_L_TEMP_PERIPH,
                                      BMS_ADC_L_TEMP_CHANNEL,
                                      temp_samples,
                                      &l_temp_raw)) {
        return 0U;
    }

    sample->inputVoltageMv = Adc_Scale_Voltage_Mv(vin_raw, BMS_ADC_VIN_GAIN_X1000);
    sample->outputVoltageMv = Adc_Scale_Voltage_Mv(vout_raw, BMS_ADC_VOUT_GAIN_X1000);
    sample->inputCurrentMa = Adc_Scale_Current_Ma(iin_raw, BMS_ADC_IIN_OFFSET_RAW, BMS_ADC_IIN_MA_PER_MV_X1000);
    sample->outputCurrentMa = Adc_Scale_Current_Ma(iout_raw, BMS_ADC_IOUT_OFFSET_RAW, BMS_ADC_IOUT_MA_PER_MV_X1000);
    sample->iinRaw = iin_raw;
    sample->vinRaw = vin_raw;
    sample->voutRaw = vout_raw;
    sample->ioutRaw = iout_raw;
    sample->mosTempRaw = mos_temp_raw;
    sample->inductorTempRaw = l_temp_raw;

    /*
     * Temperature inputs may be absent during board bring-up. Keep voltage and
     * current samples visible instead of failing the whole ADC sample group.
     */
    mos_temp_valid = Adc_Temp_Raw_Valid(mos_temp_raw);
    l_temp_valid = Adc_Temp_Raw_Valid(l_temp_raw);
    vin_present = Adc_Vin_Present(vin_raw, sample->inputVoltageMv);

    sample->mosTempX10 = (0U != mos_temp_valid) ?
                         Adc_Scale_Temp_X10(mos_temp_raw) :
                         BMS_TEMP_UNAVAILABLE_X10;
    sample->inductorTempX10 = (0U != l_temp_valid) ?
                              Adc_Scale_Temp_X10(l_temp_raw) :
                              BMS_TEMP_UNAVAILABLE_X10;
#if BMS_ENABLE_POWER_FAULT_PIN
    sample->faultOcActive = (RESET != gpio_input_bit_get(BMS_PWM_GPIO_PORT, BMS_PWM_FAULT_PIN)) ? 1U : 0U;
#else
    sample->faultOcActive = 0U;
#endif
    sample->faultBitmap = s_adc_fault;
    Adc_Sanitize_Absent_Power(sample, vin_present);
    Adc_Filter_Sample(sample);

    return 1U;
}

static uint8_t Adc_Sample_Hardware(bms_power_sample_t *sample)
{
    return Adc_Sample_Hardware_Counts(sample,
                                      ADC_VOLTAGE_AVERAGE_SAMPLES,
                                      ADC_CURRENT_AVERAGE_SAMPLES,
                                      ADC_TEMP_AVERAGE_SAMPLES);
}
#endif

void Adc_Manager_Init(void)
{
#if ADC_USE_HARDWARE
    s_adc_ready = 0U;
    s_adc_fault = 0U;
    s_filter_valid = 0U;
    memset(&s_filtered_sample, 0, sizeof(s_filtered_sample));

    rcu_periph_clock_enable(RCU_ADC0);
    rcu_periph_clock_enable(RCU_ADC1);
    Adc_Gpio_Init();
    Adc_One_Init(ADC0);
    Adc_One_Init(ADC1);
    s_adc_ready = 1U;
#endif
}

void Adc_Manager_Sample(const bms_afe_data_t *afe, bms_power_sample_t *sample)
{
    (void)afe;

    if(sample == 0) {
        return;
    }

    /*
     * 每次先清零，确保采样失败时不会残留上一轮的电压、电流或温度。
     * 上位机看到的 0 值必须同时伴随 BMS_FAULT_ADC，表示“当前无有效真实采样”。
     */
    memset(sample, 0, sizeof(*sample));

#if ADC_USE_HARDWARE
    if((s_adc_ready != 0U) && (0U != Adc_Sample_Hardware(sample))) {
        return;
    }
#endif

    sample->mosTempX10 = BMS_TEMP_UNAVAILABLE_X10;
    sample->inductorTempX10 = BMS_TEMP_UNAVAILABLE_X10;
    sample->faultBitmap |= BMS_FAULT_ADC;
}

void Adc_Manager_Sample_Fast(bms_power_sample_t *sample)
{
    if(sample == 0) {
        return;
    }

    memset(sample, 0, sizeof(*sample));

#if ADC_USE_HARDWARE
    if((s_adc_ready != 0U) &&
       (0U != Adc_Sample_Hardware_Counts(sample,
                                          BMS_ADC_FAST_VOLTAGE_AVERAGE_SAMPLES,
                                          BMS_ADC_FAST_CURRENT_AVERAGE_SAMPLES,
                                          1U))) {
        return;
    }
#endif

    sample->mosTempX10 = BMS_TEMP_UNAVAILABLE_X10;
    sample->inductorTempX10 = BMS_TEMP_UNAVAILABLE_X10;
    sample->faultBitmap |= BMS_FAULT_ADC;
}
