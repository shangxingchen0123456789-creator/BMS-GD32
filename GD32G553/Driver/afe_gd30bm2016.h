#ifndef AFE_GD30BM2016_H
#define AFE_GD30BM2016_H

#include "bms_types.h"

#define AFE_GD30BM2016_FET_STATUS_CHG         (1U << 0)
#define AFE_GD30BM2016_FET_STATUS_PCHG        (1U << 1)
#define AFE_GD30BM2016_FET_STATUS_DSG         (1U << 2)
#define AFE_GD30BM2016_FET_STATUS_PDSG        (1U << 3)
#define AFE_GD30BM2016_FET_STATUS_DCHG_PIN    (1U << 4)
#define AFE_GD30BM2016_FET_STATUS_DDSG_PIN    (1U << 5)
#define AFE_GD30BM2016_FET_STATUS_ALERT_PIN   (1U << 6)
#define AFE_GD30BM2016_FET_STATUS_DFETOFF_HIGH (1U << 7)

/*
 * 应用任务使用的 AFE 对外接口。
 * 本工程正式固件只接受 BM2016 真实采样；通信失败时返回故障位和清零数据，
 * 不生成模拟电芯电压，避免上位机误判为真实电池状态。
 */
void Afe_Gd30bm2016_Init(void);
void Afe_Gd30bm2016_Poll(bms_afe_data_t *data);
void Afe_Gd30bm2016_Set_Balance(uint16_t balance_bitmap);
void Afe_Gd30bm2016_Force_Path_Off_Fast(void);
void Afe_Gd30bm2016_Fets_Off(void);
uint8_t Afe_Gd30bm2016_Fets_On(uint8_t *status);
uint8_t Afe_Gd30bm2016_Recover_Protections(void);
uint8_t Afe_Gd30bm2016_Set_Fet_Mask(uint8_t fet_mask, uint8_t *status);
uint8_t Afe_Gd30bm2016_Read_Fet_Status(uint8_t *status);
uint8_t Afe_Gd30bm2016_Read_Path_Voltages(uint16_t *stack_mv, uint16_t *pack_mv, uint16_t *ld_mv);
uint8_t Afe_Gd30bm2016_Alert_Active(void);

typedef struct {
    uint8_t configFailStage;
    uint8_t configFailStep;
    uint8_t configFailIndex;
    uint8_t cfgupdateSeen;
    uint8_t i2cAddrWrite;
    uint8_t lastReg;
    uint8_t lastCrcOk;
    uint8_t lastCrcRx;
    uint8_t lastCrcCalc;
    uint16_t configFailReg;
    uint16_t lastBatteryStatus;
    uint16_t probeVcellMode;
    uint16_t rawCell9Mv;
    uint16_t rawCell16Mv;
    uint16_t stackMinusCell1_8Mv;
    uint8_t safetyStatusA;
    uint8_t safetyStatusB;
    uint8_t safetyStatusC;
    uint8_t manufacturingStatus;
    uint8_t fetOptions;
} afe_gd30bm2016_debug_t;

void Afe_Gd30bm2016_Get_Debug(afe_gd30bm2016_debug_t *debug);

#endif
