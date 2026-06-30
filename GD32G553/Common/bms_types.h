#ifndef BMS_TYPES_H
#define BMS_TYPES_H

#include <stdint.h>

#define BMS_CELL_COUNT                         9U
#define BMS_AFE_TEMP_COUNT                     4U
#define BMS_TEMP_COUNT                         6U
#define BMS_TEMP_UNAVAILABLE_X10               0x7FFF
#define BMS_TEMP_MIN_VALID_X10                 (-400)
#define BMS_TEMP_MAX_VALID_X10                 1250

#define BMS_DEFAULT_TARGET_VOLTAGE_MV          37800U
/* 2000 mAh 18650 9S：CC 取 0.5C，截止取 0.05C，与功率板输入功率上限自洽。 */
#define BMS_DEFAULT_TARGET_CURRENT_MA          1000U
#define BMS_DEFAULT_CUTOFF_CURRENT_MA          100U
#define BMS_DEFAULT_CELL_OVP_MV                4200U
#define BMS_DEFAULT_CELL_UVP_MV                3000U
#define BMS_DEFAULT_TEMP_OTP_X10               600
#define BMS_DEFAULT_BALANCE_DELTA_MV           30U

/* 充电状态值会直接通过协议发送给上位机。 */
typedef enum {
    BMS_CHARGE_STATE_IDLE = 0,
    BMS_CHARGE_STATE_PRECHECK = 1,
    BMS_CHARGE_STATE_TRICKLE = 2,
    BMS_CHARGE_STATE_CC = 3,
    BMS_CHARGE_STATE_CV = 4,
    BMS_CHARGE_STATE_DONE = 5,
    BMS_CHARGE_STATE_FAULT = 6,
    BMS_CHARGE_STATE_DIGITAL_POWER = 7,
    BMS_CHARGE_STATE_DIGITAL_POWER_WAIT_VOUT = 8
} bms_charge_state_t;

/* 手动模式强制指定阶段，AUTO 模式由状态机自动选择阶段。 */
typedef enum {
    BMS_CHARGE_MODE_AUTO = 0,
    BMS_CHARGE_MODE_TRICKLE = 1,
    BMS_CHARGE_MODE_CC = 2,
    BMS_CHARGE_MODE_CV = 3,
    BMS_CHARGE_MODE_DIGITAL_POWER = 4
} bms_charge_mode_t;

typedef enum {
    BMS_WORK_MODE_BMS = 0,
    BMS_WORK_MODE_DIGITAL_POWER = 1
} bms_work_mode_t;

/* 故障位图必须与固件协议和上位机故障文本保持一致。 */
typedef enum {
    BMS_FAULT_CELL_OVP            = (1UL << 0),
    BMS_FAULT_CELL_UVP            = (1UL << 1),
    BMS_FAULT_PACK_OVP            = (1UL << 2),
    BMS_FAULT_CHARGE_OCP          = (1UL << 3),
    BMS_FAULT_OTP                 = (1UL << 4),
    BMS_FAULT_AFE_COMM            = (1UL << 5),
    BMS_FAULT_ADC                 = (1UL << 6),
    BMS_FAULT_INPUT_UV            = (1UL << 7),
    BMS_FAULT_BATTERY_DISCONNECT  = (1UL << 8),
    BMS_FAULT_EMERGENCY_STOP      = (1UL << 9),
    BMS_FAULT_SHORT_CIRCUIT       = (1UL << 10),
    BMS_FAULT_FUSE                = (1UL << 11),
    BMS_FAULT_AFE_PROTECTION      = (1UL << 12),
    BMS_FAULT_DIGITAL_POWER_RESTART_TIMEOUT = (1UL << 13)
} bms_fault_bit_t;

/* 上位机可调充电参数，电压/电流单位分别为 mV 和 mA。 */
typedef struct {
    uint16_t targetVoltageMv;        /* 目标充电总电压，单位 mV。 */
    uint16_t targetCurrentMa;        /* 目标充电电流上限，单位 mA。 */
    uint16_t cutoffCurrentMa;        /* 恒压阶段完成判定电流，单位 mA。 */
    uint16_t cellOvpMv;              /* 单体过压保护阈值，单位 mV。 */
    uint16_t cellUvpMv;              /* 单体欠压保护阈值，单位 mV。 */
    int16_t tempOtpX10;              /* 过温保护阈值，单位 0.1 摄氏度。 */
    uint16_t balanceDeltaMv;         /* 允许开启均衡的单体压差阈值，单位 mV。 */
} bms_charge_parameters_t;

/* 归一化后的 BM2016 数据，温度单位为 0.1 摄氏度。 */
typedef struct {
    uint16_t cellMv[BMS_CELL_COUNT];          /* 各串单体电压，单位 mV。 */
    uint16_t cellMaxMv;                       /* 当前最高单体电压，单位 mV。 */
    uint16_t cellMinMv;                       /* 当前最低单体电压，单位 mV。 */
    uint16_t cellDeltaMv;                     /* 最高与最低单体电压差，单位 mV。 */
    uint16_t packVoltageMv;                   /* 电池包总电压，单位 mV。 */
    int16_t batteryCurrentMa;                 /* R71/BM2016 电池侧电流，正值充电、负值放电，单位 mA。 */
    int16_t temperaturesX10[BMS_AFE_TEMP_COUNT];  /* AFE 侧温度通道，单位 0.1 摄氏度。 */
    uint32_t faultBitmap;                     /* AFE 采样或芯片状态产生的故障位图。 */
} bms_afe_data_t;

/* 功率级 ADC 经标定和保护检查后的采样结果。 */
typedef struct {
    uint16_t inputVoltageMv;         /* 充电器输入端电压 Vin，单位 mV。 */
    uint16_t outputVoltageMv;        /* 充电器输出端/电池端电压 Vout，单位 mV。 */
    int16_t inputCurrentMa;          /* R11 输入电流 Iin，用于输入保护和功率限制，单位 mA。 */
    int16_t outputCurrentMa;         /* R12 输出电流 Iout，用于功率板快环控制和输出保护，单位 mA。 */
    int16_t mosTempX10;              /* MOS 管温度，单位 0.1 摄氏度。 */
    int16_t inductorTempX10;         /* 电感温度，单位 0.1 摄氏度。 */
    uint8_t faultOcActive;           /* 硬件过流 FAULT_OC 是否触发，1 表示触发。 */
    uint16_t iinRaw;                 /* ADC_Iin 12-bit raw count before input-present filtering. */
    uint16_t vinRaw;                 /* ADC_Vin 12-bit raw count before input-present filtering. */
    uint16_t voutRaw;                /* ADC_Vout 12-bit raw count before input-present filtering. */
    uint16_t ioutRaw;                /* ADC_Iout 12-bit raw count before input-present filtering. */
    uint16_t mosTempRaw;             /* ADC_MOS_TEMP 12-bit raw count before input-present filtering. */
    uint16_t inductorTempRaw;        /* ADC_L_TEMP 12-bit raw count before input-present filtering. */
    uint32_t faultBitmap;            /* ADC/功率采样链路产生的故障位图。 */
} bms_power_sample_t;

/* 协议上报状态源，字段与 Comm/bms_protocol.c 的负载打包顺序对应。 */
typedef struct {
    uint32_t timestampMs;                     /* 状态快照时间戳，单位 ms。 */
    uint16_t packVoltageMv;                   /* 上报用电池包总电压，单位 mV。 */
    int16_t chargeCurrentMa;                  /* 上报用 R71/BM2016 电池侧电流，正值充电、负值放电，单位 mA。 */
    uint16_t inputVoltageMv;                  /* 上报用功率板输入电压 Vin，单位 mV。 */
    uint16_t outputVoltageMv;                 /* 上报用充电电压 Vout，单位 mV。 */
    uint16_t dutyX100;                        /* 当前控制占空比，百分比 * 100。 */
    uint16_t socX10;                          /* 荷电状态 SOC，百分比 * 10。 */
    uint8_t chargeState;                      /* 当前充电状态，取值见 bms_charge_state_t。 */
    uint8_t chargeMode;                       /* 当前充电模式，取值见 bms_charge_mode_t。 */
    uint32_t faultBitmap;                     /* 汇总故障位图，取值见 bms_fault_bit_t。 */
    int16_t temperaturesX10[BMS_TEMP_COUNT];  /* 上报用温度通道，单位 0.1 摄氏度。 */
    uint16_t cellMv[BMS_CELL_COUNT];          /* 上报用各串单体电压，单位 mV。 */
    uint16_t cellMaxMv;                       /* 上报用最高单体电压，单位 mV。 */
    uint16_t cellMinMv;                       /* 上报用最低单体电压，单位 mV。 */
    uint16_t cellDeltaMv;                     /* 上报用单体最大压差，单位 mV。 */
    uint16_t balanceBitmap;                   /* 均衡位图，bit0 对应第 1 串电芯。 */
    uint8_t workMode;                         /* Current top-level work mode, see bms_work_mode_t. */
} bms_status_t;

#endif
