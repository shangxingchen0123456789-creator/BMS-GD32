#ifndef AFE_GD30BM2016_REGS_H
#define AFE_GD30BM2016_REGS_H

#include <stdint.h>

/*
 * GD30BM2016 寄存器映射。
 *
 * 来源：BM2016 资料包：
 * - AN-GD30BM2016 Register Maps Rev.B
 * - AN-GD30BM2016 Commands and Subcommands Rev.A
 * - GD30BM2016_DEMO_V1.0/Application/Include/gd30bm2016.h
 *
 * 所有芯片可见地址集中放在这里，让硬件驱动文件保持简洁。
 */

#define GD30BM2016_I2C_ADDR_7BIT                         0x10U
#define GD30BM2016_I2C_ADDR_WRITE                        0x20U
#define GD30BM2016_I2C_ADDR_READ                         0x21U

/* CRC8 parameters used by the BM2016 I2C packet checksum path. */
#define GD30BM2016_CRC8_POLY                             0x07U
#define GD30BM2016_CRC8_INIT                             0x00U

/* 直接寄存器命令。 */
#define GD30BM2016_DIR_CONTROL_STATUS                    0x00U
#define GD30BM2016_DIR_SAFETY_ALERT_A                    0x02U
#define GD30BM2016_DIR_SAFETY_STATUS_A                   0x03U
#define GD30BM2016_DIR_SAFETY_ALERT_B                    0x04U
#define GD30BM2016_DIR_SAFETY_STATUS_B                   0x05U
#define GD30BM2016_DIR_SAFETY_ALERT_C                    0x06U
#define GD30BM2016_DIR_SAFETY_STATUS_C                   0x07U
#define GD30BM2016_DIR_PF_ALERT_A                        0x0AU
#define GD30BM2016_DIR_PF_STATUS_A                       0x0BU
#define GD30BM2016_DIR_PF_ALERT_B                        0x0CU
#define GD30BM2016_DIR_PF_STATUS_B                       0x0DU
#define GD30BM2016_DIR_PF_ALERT_C                        0x0EU
#define GD30BM2016_DIR_PF_STATUS_C                       0x0FU
#define GD30BM2016_DIR_PF_ALERT_D                        0x10U
#define GD30BM2016_DIR_PF_STATUS_D                       0x11U
#define GD30BM2016_DIR_BATTERY_STATUS                    0x12U
#define GD30BM2016_DIR_CELL1_VOLTAGE                     0x14U
#define GD30BM2016_DIR_CELL2_VOLTAGE                     0x16U
#define GD30BM2016_DIR_CELL3_VOLTAGE                     0x18U
#define GD30BM2016_DIR_CELL4_VOLTAGE                     0x1AU
#define GD30BM2016_DIR_CELL5_VOLTAGE                     0x1CU
#define GD30BM2016_DIR_CELL6_VOLTAGE                     0x1EU
#define GD30BM2016_DIR_CELL7_VOLTAGE                     0x20U
#define GD30BM2016_DIR_CELL8_VOLTAGE                     0x22U
#define GD30BM2016_DIR_CELL9_VOLTAGE                     0x24U
#define GD30BM2016_DIR_CELL10_VOLTAGE                    0x28U
#define GD30BM2016_DIR_CELL11_VOLTAGE                    0x2AU
#define GD30BM2016_DIR_CELL12_VOLTAGE                    0x2CU
#define GD30BM2016_DIR_CELL13_VOLTAGE                    0x2EU
#define GD30BM2016_DIR_CELL14_VOLTAGE                    0x30U
#define GD30BM2016_DIR_CELL15_VOLTAGE                    0x32U
#define GD30BM2016_DIR_STACK_VOLTAGE                     0x34U
#define GD30BM2016_DIR_PACK_PIN_VOLTAGE                  0x36U
#define GD30BM2016_DIR_LD_PIN_VOLTAGE                    0x38U
#define GD30BM2016_DIR_CC2_CURRENT                       0x3AU
#define GD30BM2016_DIR_CELL16_VOLTAGE                    0x3CU
#define GD30BM2016_DIR_SUBCMD                            0x3EU
#define GD30BM2016_DIR_SUBCMD_DATA                       0x40U
#define GD30BM2016_DIR_ALARM_STATUS                      0x62U
#define GD30BM2016_DIR_ALARM_RAW_STATUS                  0x64U
#define GD30BM2016_DIR_ALARM_ENABLE                      0x66U
#define GD30BM2016_DIR_INT_TEMPERATURE                   0x68U
#define GD30BM2016_DIR_CFETOFF_TEMPERATURE               0x6AU
#define GD30BM2016_DIR_DFETOFF_TEMPERATURE               0x6CU
#define GD30BM2016_DIR_ALERT_TEMPERATURE                 0x6EU
#define GD30BM2016_DIR_TS1_TEMPERATURE                   0x70U
#define GD30BM2016_DIR_TS2_TEMPERATURE                   0x72U
#define GD30BM2016_DIR_TS3_TEMPERATURE                   0x74U
#define GD30BM2016_DIR_HDQ_TEMPERATURE                   0x76U
#define GD30BM2016_DIR_DCHG_TEMPERATURE                  0x78U
#define GD30BM2016_DIR_DDSG_TEMPERATURE                  0x7AU
#define GD30BM2016_DIR_FET_STATUS                        0x7FU
#define GD30BM2016_DIR_SUBCMD_CHECKSUM_LEN               0x60U
#define GD30BM2016_DIR_SUBCMD_READ_LEN                   0x61U

#define GD30BM2016_CELL_VOLTAGE_LSB_MV                   1U
#define GD30BM2016_STACK_PACK_LD_LSB_MV                  8U

/* 兼容第一版驱动草稿中使用过的旧名称。 */
#define GD30BM2016_REG_CELL1_VOLTAGE_L                   GD30BM2016_DIR_CELL1_VOLTAGE
#define GD30BM2016_CELL_VOLTAGE_STRIDE                   2U
#define GD30BM2016_REG_SAFETY_ALERT                      GD30BM2016_DIR_SAFETY_ALERT_A
#define GD30BM2016_REG_SAFETY_STATUS                     GD30BM2016_DIR_SAFETY_STATUS_A
#define GD30BM2016_REG_PF_ALERT                          GD30BM2016_DIR_PF_ALERT_A
#define GD30BM2016_REG_PF_STATUS                         GD30BM2016_DIR_PF_STATUS_A

/* 无数据负载的子命令。 */
#define GD30BM2016_SUBCMD_EXIT_DEEPSLEEP                 0x0300U
#define GD30BM2016_SUBCMD_DEEPSLEEP                      0x0304U
#define GD30BM2016_SUBCMD_SHUTDOWN                       0x0308U
#define GD30BM2016_SUBCMD_RESET                          0x030CU
#define GD30BM2016_SUBCMD_PDSGTEST                       0x0310U
#define GD30BM2016_SUBCMD_FUSE_TOGGLE                    0x0314U
#define GD30BM2016_SUBCMD_PCHGTEST                       0x0318U
#define GD30BM2016_SUBCMD_FET_ENABLE                     0x0324U
#define GD30BM2016_SUBCMD_PF_ENABLE                      0x0328U
#define GD30BM2016_SUBCMD_PF_RESET                       0x0029U
#define GD30BM2016_SUBCMD_SEAL                           0x032CU
#define GD30BM2016_SUBCMD_RESET_PASSQ                    0x0330U
#define GD30BM2016_SUBCMD_PTO_RECOVER                    0x0334U
#define GD30BM2016_SUBCMD_SET_CFGUPDATE                  0x0338U
#define GD30BM2016_SUBCMD_EXIT_CFGUPDATE                 0x033CU
#define GD30BM2016_SUBCMD_DSG_PDSG_OFF                   0x0340U
#define GD30BM2016_SUBCMD_CHG_PCHG_OFF                   0x0344U
#define GD30BM2016_SUBCMD_ALL_FETS_OFF                   0x0348U
#define GD30BM2016_SUBCMD_ALL_FETS_ON                    0x034CU
#define GD30BM2016_SUBCMD_SLEEP_ENABLE                   0x0350U
#define GD30BM2016_SUBCMD_SLEEP_DISABLE                  0x0354U
#define GD30BM2016_SUBCMD_OCDL_RECOVER                   0x0358U
#define GD30BM2016_SUBCMD_SCDL_RECOVER                   0x035CU
#define GD30BM2016_SUBCMD_LOAD_DETECT_RESTART            0x0360U
#define GD30BM2016_SUBCMD_LOAD_DETECT_ON                 0x0364U
#define GD30BM2016_SUBCMD_LOAD_DETECT_OFF                0x0368U
#define GD30BM2016_SUBCMD_CFETOFF_LO                     0x036CU
#define GD30BM2016_SUBCMD_DFETOFF_LO                     0x0370U
#define GD30BM2016_SUBCMD_ALERT_LO                       0x0374U
#define GD30BM2016_SUBCMD_HDQ_LO                         0x0378U
#define GD30BM2016_SUBCMD_DCHG_LO                        0x037CU
#define GD30BM2016_SUBCMD_DDSG_LO                        0x0380U
#define GD30BM2016_SUBCMD_CFETOFF_HI                     0x0384U
#define GD30BM2016_SUBCMD_DFETOFF_HI                     0x0388U
#define GD30BM2016_SUBCMD_ALERT_HI                       0x038CU
#define GD30BM2016_SUBCMD_HDQ_HI                         0x0390U
#define GD30BM2016_SUBCMD_DCHG_HI                        0x0394U
#define GD30BM2016_SUBCMD_DDSG_HI                        0x0398U
#define GD30BM2016_SUBCMD_PF_FORCE_A                     0x039CU
#define GD30BM2016_SUBCMD_PF_FORCE_B                     0x03A0U
#define GD30BM2016_SUBCMD_SWAP_COMM_MODE                 0x03A4U
#define GD30BM2016_SUBCMD_SWAP_TO_I2C                    0x03A8U
#define GD30BM2016_SUBCMD_SLEEP                          0x03B4U
#define GD30BM2016_SUBCMD_CHIP_RESET                     0x03CCU

/* 带数据负载的子命令。 */
#define GD30BM2016_CMD_DEVICE_NUMBER                     0x0000U
#define GD30BM2016_CMD_HW_VERSION                        0x000CU
#define GD30BM2016_CMD_SYS_STATE                         0x0010U
#define GD30BM2016_CMD_OTP_STATE                         0x0012U
#define GD30BM2016_CMD_ADC_STATE                         0x0013U
#define GD30BM2016_CMD_PT_STATE                          0x0014U
#define GD30BM2016_CMD_CB_STATE                          0x0015U
#define GD30BM2016_CMD_UNSEAL_KEYSTEP1                   0x0020U
#define GD30BM2016_CMD_UNSEAL_KEYSTEP2                   0x0022U
#define GD30BM2016_CMD_FULL_ACCESS_KEYSTEP1              0x0024U
#define GD30BM2016_CMD_FULL_ACCESS_KEYSTEP2              0x0026U
#define GD30BM2016_CMD_SAVED_PF_STATUS_A                 0x0028U
#define GD30BM2016_CMD_SAVED_PF_STATUS_B                 0x0029U
#define GD30BM2016_CMD_SAVED_PF_STATUS_C                 0x002AU
#define GD30BM2016_CMD_SAVED_PF_STATUS_D                 0x002BU
#define GD30BM2016_CMD_FUSE_FLAG                         0x002CU
#define GD30BM2016_CMD_MANUFACTURING_STATUS              0x0030U
#define GD30BM2016_CMD_MAX_CELL_VOLTAGE                  0x0104U
#define GD30BM2016_CMD_MIN_CELL_VOLTAGE                  0x0106U
#define GD30BM2016_CMD_BATTERY_VOLTAGE_SUM               0x0108U
#define GD30BM2016_CMD_CELL_TEMP                         0x010AU
#define GD30BM2016_CMD_FET_TEMP                          0x010CU
#define GD30BM2016_CMD_MAX_CELL_TEMP                     0x010EU
#define GD30BM2016_CMD_MIN_CELL_TEMP                     0x0110U
#define GD30BM2016_CMD_CC3_CURRENT                       0x0114U
#define GD30BM2016_CMD_CC1_CURRENT                       0x0116U
#define GD30BM2016_CMD_CC2_COUNTS                        0x0118U
#define GD30BM2016_CMD_ACCUM_CHARGE                      0x0120U
#define GD30BM2016_CMD_ACCUM_CHARGE_FRACTION             0x0124U
#define GD30BM2016_CMD_ACCUM_TIME                        0x0128U
#define GD30BM2016_CMD_CELL_BALANCING_ACTIVE_CELLS       0x0200U
#define GD30BM2016_CMD_CELL_BALANCING_SET_LEVEL          0x0204U
#define GD30BM2016_CMD_CELL_BALANCING_PRESENT_TIME       0x0208U
#define GD30BM2016_CMD_FET_CONTROL                       0x0250U
#define GD30BM2016_CMD_REG12_CONTROL                     0x0254U
#define GD30BM2016_CMD_CC2_AVG_COUNTS                    0x0260U
#define GD30BM2016_CMD_PACKPIN_ADC_COUNTS                0x0268U
#define GD30BM2016_CMD_TOP_OF_STACK_ADC_COUNTS           0x026AU
#define GD30BM2016_CMD_LDPIN_ADC_COUNTS                  0x026CU
#define GD30BM2016_CMD_ACCESS_KEYSTEP1                   0x0280U
#define GD30BM2016_CMD_ACCESS_KEYSTEP2                   0x0282U

/* 系统类 Data Memory 寄存器。 */
#define GD30BM2016_DM_POWER_CONFIG                       0x8000U
#define GD30BM2016_DM_REG12_CONFIG                       0x8002U
#define GD30BM2016_DM_REG0_CONFIG                        0x8003U
#define GD30BM2016_DM_HWD_REGULATOR_OPTIONS              0x8004U
#define GD30BM2016_DM_COMM_TYPE                          0x8005U
#define GD30BM2016_DM_I2C_ADDRESS                        0x8006U
#define GD30BM2016_DM_COMM_IDLE_TIME                     0x8008U
#define GD30BM2016_DM_LOWV_SHUTDOWN_DELAY                0x8009U
#define GD30BM2016_DM_SHUTDOWN_CELL_VOLTAGE              0x800AU
#define GD30BM2016_DM_SHUTDOWN_STACK_VOLTAGE             0x800CU
#define GD30BM2016_DM_SHUTDOWN_TEMPERATURE               0x800EU
#define GD30BM2016_DM_SHUTDOWN_TEMPERATURE_DELAY         0x8010U
#define GD30BM2016_DM_SHUTDOWN_COMMAND_DELAY             0x8011U
#define GD30BM2016_DM_AUTO_SHUTDOWN_TIME                 0x8012U
#define GD30BM2016_DM_MANUFACTURING                      0x8013U
#define GD30BM2016_DM_SLEEP_CURRENT                      0x8014U
#define GD30BM2016_DM_SLEEP_HYSTERESIS_TIME              0x8016U
#define GD30BM2016_DM_VOLTAGE_TIME                       0x8017U
#define GD30BM2016_DM_WAKE_COMPARATOR_CURRENT            0x8018U
#define GD30BM2016_DM_SLEEP_CHARGER_VOLTAGE_THRESHOLD    0x801AU
#define GD30BM2016_DM_SLEEP_CHARGER_PACKTOS_DELTA        0x801CU
#define GD30BM2016_DM_CELL1_INTERCONNECT                 0x801EU
#define GD30BM2016_DM_UNSEAL_KEYSTEP1                    0x8040U
#define GD30BM2016_DM_UNSEAL_KEYSTEP2                    0x8042U
#define GD30BM2016_DM_FULL_ACCESS_KEYSTEP1               0x8044U
#define GD30BM2016_DM_FULL_ACCESS_KEYSTEP2               0x8046U
#define GD30BM2016_DM_SECURITY_SETTINGS                  0x8048U
#define GD30BM2016_DM_ADC_CLK_CFG                        0x8049U
#define GD30BM2016_DM_OTP_CTL                            0x804AU
#define GD30BM2016_DM_WDG_EN                             0x804CU
#define GD30BM2016_DM_FET_OFF_DELAY                      0x804DU
#define GD30BM2016_DM_SCAN_MODE                          0x804EU

/* 引脚与温度输入配置寄存器。 */
#define GD30BM2016_DM_CFETOFF_PIN_CONFIG                 0x8200U
#define GD30BM2016_DM_DFETOFF_PIN_CONFIG                 0x8201U
#define GD30BM2016_DM_ALERT_PIN_CONFIG                   0x8202U
#define GD30BM2016_DM_SCL_PIN_CONFIG                     0x8204U
#define GD30BM2016_DM_SDA_PIN_CONFIG                     0x8205U
#define GD30BM2016_DM_HDQ_PIN_CONFIG                     0x8206U
#define GD30BM2016_DM_DCHG_PIN_CONFIG                    0x8207U
#define GD30BM2016_DM_DDSG_PIN_CONFIG                    0x8208U
#define GD30BM2016_DM_TS1_CONFIG                         0x8209U
#define GD30BM2016_DM_TS23_CONFIG                        0x820AU
#define GD30BM2016_DM_PAD_CFG                            0x8210U

/* ADC 与校准寄存器。 */
#define GD30BM2016_DM_CELL1_GAIN                         0x8400U
#define GD30BM2016_DM_PACK_GAIN                          0x8422U
#define GD30BM2016_DM_TOS_GAIN                           0x8424U
#define GD30BM2016_DM_LD_GAIN                            0x8426U
#define GD30BM2016_DM_ADC_GAIN                           0x8428U
#define GD30BM2016_DM_CC_GAIN                            0x842CU
#define GD30BM2016_DM_CC2_GAIN                           0x8430U
#define GD30BM2016_DM_CC2_OFFSET                         0x8432U
#define GD30BM2016_DM_VCELL_OFFSET                       0x8434U
#define GD30BM2016_DM_VDIV_OFFSET                        0x8436U
#define GD30BM2016_DM_ADC_OFFSET                         0x8438U
#define GD30BM2016_DM_CC_OFFSET_SAMPLES                  0x843AU
#define GD30BM2016_DM_BOARD_OFFSET                       0x843CU
#define GD30BM2016_DM_SLEEP_MODE_OFFSET                  0x843EU
#define GD30BM2016_DM_INT_GAIN                           0x8440U
#define GD30BM2016_DM_INTERNAL_TEMP_OFFSET               0x8442U
#define GD30BM2016_DM_INT_MAXIMUM_AD                     0x8444U
#define GD30BM2016_DM_INT_MAXIMUM_TEMP                   0x8446U
#define GD30BM2016_DM_T18K_COEFF_A1                      0x8448U
#define GD30BM2016_DM_T18K_COEFF_A2                      0x844AU
#define GD30BM2016_DM_T18K_COEFF_A3                      0x844CU
#define GD30BM2016_DM_T18K_COEFF_A4                      0x844EU
#define GD30BM2016_DM_T180K_COEFF_A1                     0x8450U
#define GD30BM2016_DM_T180K_COEFF_A2                     0x8452U
#define GD30BM2016_DM_T180K_COEFF_A3                     0x8454U
#define GD30BM2016_DM_T180K_COEFF_A4                     0x8456U
#define GD30BM2016_DM_CUSTOM_COEFF_A1                    0x8458U
#define GD30BM2016_DM_CUSTOM_COEFF_A2                    0x845AU
#define GD30BM2016_DM_CUSTOM_COEFF_A3                    0x845CU
#define GD30BM2016_DM_CUSTOM_COEFF_A4                    0x845EU
#define GD30BM2016_DM_COULOMB_COUNTER_DEADBAND           0x8460U
#define GD30BM2016_DM_CC3_SAMPLES                        0x8461U
#define GD30BM2016_DM_ADC_CFG                            0x8462U
#define GD30BM2016_DM_ADC_CC1_SEL                        0x8463U
#define GD30BM2016_DM_ADC_CC2_RSENSE                     0x8464U
#define GD30BM2016_DM_CODE_GAIN                          0x8466U
#define GD30BM2016_DM_CODE_OFFSET                        0x8468U
#define GD30BM2016_DM_ADC_EN                             0x846AU

/* 保护与 FET 相关寄存器。 */
#define GD30BM2016_DM_SF_CTRL                            0x8800U
#define GD30BM2016_DM_LOAD_REMOVE_TH                     0x8802U
#define GD30BM2016_DM_MIN_BLOW_FUSE_VOLTAGE              0x8804U
#define GD30BM2016_DM_FUSE_BLOW_TIMEOUT                  0x8806U
#define GD30BM2016_DM_DA_CONFIG                          0x8807U
#define GD30BM2016_DM_VCELL_MODE                         0x8808U
#define GD30BM2016_DM_PROTECTION_CONFIGURATION           0x880AU
#define GD30BM2016_DM_ENABLED_PROTECTIONS_A              0x880CU
#define GD30BM2016_DM_ENABLED_PROTECTIONS_B              0x880DU
#define GD30BM2016_DM_ENABLED_PROTECTIONS_C              0x880EU
#define GD30BM2016_DM_CHG_FET_PROTECTIONS_A              0x880FU
#define GD30BM2016_DM_CHG_FET_PROTECTIONS_B              0x8810U
#define GD30BM2016_DM_CHG_FET_PROTECTIONS_C              0x8811U
#define GD30BM2016_DM_DSG_FET_PROTECTIONS_A              0x8812U
#define GD30BM2016_DM_DSG_FET_PROTECTIONS_B              0x8813U
#define GD30BM2016_DM_DSG_FET_PROTECTIONS_C              0x8814U
#define GD30BM2016_DM_SF_ALERT_MASK_A                    0x8815U
#define GD30BM2016_DM_SF_ALERT_MASK_B                    0x8816U
#define GD30BM2016_DM_SF_ALERT_MASK_C                    0x8817U
#define GD30BM2016_DM_PF_ALERT_MASK_A                    0x8818U
#define GD30BM2016_DM_PF_ALERT_MASK_B                    0x8819U
#define GD30BM2016_DM_PF_ALERT_MASK_C                    0x881AU
#define GD30BM2016_DM_PF_ALERT_MASK_D                    0x881BU
#define GD30BM2016_DM_BODY_DIODE_THRESHOLD               0x881CU
#define GD30BM2016_DM_DEFAULT_ALARM_MASK                 0x881EU
#define GD30BM2016_DM_ENABLED_PF_A                       0x8820U
#define GD30BM2016_DM_ENABLED_PF_B                       0x8821U
#define GD30BM2016_DM_ENABLED_PF_C                       0x8822U
#define GD30BM2016_DM_ENABLED_PF_D                       0x8823U
#define GD30BM2016_DM_FET_OPTIONS                        0x8824U
#define GD30BM2016_DM_CHG_PUMP_CONTROL                   0x8825U
#define GD30BM2016_DM_PRECHARGE_START_VOLTAGE            0x8826U
#define GD30BM2016_DM_PRECHARGE_STOP_VOLTAGE             0x8828U
#define GD30BM2016_DM_PREDISCHARGE_TIMEOUT               0x882AU
#define GD30BM2016_DM_PREDISCHARGE_STOP_DELTA            0x882BU
#define GD30BM2016_DM_CUV_THRESHOLD                      0x882CU
#define GD30BM2016_DM_CUV_DELAY                          0x882EU
#define GD30BM2016_DM_CUV_RECOVERY_HYSTERESIS            0x8830U
#define GD30BM2016_DM_COV_THRESHOLD                      0x8832U
#define GD30BM2016_DM_COV_DELAY                          0x8834U
#define GD30BM2016_DM_COV_RECOVERY_HYSTERESIS            0x8836U
#define GD30BM2016_DM_COVL_LATCH_LIMIT                   0x8837U
#define GD30BM2016_DM_COVL_COUNTER_DEC_DELAY             0x8838U
#define GD30BM2016_DM_COVL_RECOVERY_TIME                 0x8839U
#define GD30BM2016_DM_OCC_THRESHOLD                      0x883AU
#define GD30BM2016_DM_OCC_DELAY                          0x883CU
#define GD30BM2016_DM_OCC_RECOVERY_THRESHOLD             0x8840U
#define GD30BM2016_DM_OCC_PACKTOS_DELTA                  0x8842U
#define GD30BM2016_DM_OCD1_THRESHOLD                     0x8844U
#define GD30BM2016_DM_OCD1_DELAY                         0x8846U
#define GD30BM2016_DM_OCD2_THRESHOLD                     0x8848U
#define GD30BM2016_DM_OCD2_DELAY                         0x884AU
#define GD30BM2016_DM_SCD_THRESHOLD                      0x884BU
#define GD30BM2016_DM_SCD_DELAY                          0x884CU
#define GD30BM2016_DM_SCD_RECOVERY_TIME                  0x884DU
#define GD30BM2016_DM_OCD3_THRESHOLD                     0x884EU
#define GD30BM2016_DM_OCD3_DELAY                         0x8850U
#define GD30BM2016_DM_OCD_RECOVERY_THRESHOLD             0x8852U
#define GD30BM2016_DM_OCDL_LATCH_LIMIT                   0x8854U
#define GD30BM2016_DM_OCDL_COUNTER_DEC_DELAY             0x8855U
#define GD30BM2016_DM_OCDL_RECOVERY_TIME                 0x8856U
#define GD30BM2016_DM_OCDL_RECOVERY_THRESHOLD            0x8858U
#define GD30BM2016_DM_SCDL_LATCH_LIMIT                   0x885AU
#define GD30BM2016_DM_SCDL_COUNTER_DEC_DELAY             0x885BU
#define GD30BM2016_DM_SCDL_RECOVERY_TIME                 0x885CU
#define GD30BM2016_DM_SCDL_RECOVERY_THRESHOLD            0x885EU
#define GD30BM2016_DM_OTC_THRESHOLD                      0x8860U
#define GD30BM2016_DM_OTC_DELAY                          0x8861U
#define GD30BM2016_DM_OTC_RECOVERY_THRESHOLD             0x8862U
#define GD30BM2016_DM_OTD_THRESHOLD                      0x8863U
#define GD30BM2016_DM_OTD_DELAY                          0x8864U
#define GD30BM2016_DM_OTD_RECOVERY_THRESHOLD             0x8865U
#define GD30BM2016_DM_OTF_THRESHOLD                      0x8866U
#define GD30BM2016_DM_OTF_DELAY                          0x8867U
#define GD30BM2016_DM_OTF_RECOVERY_THRESHOLD             0x8868U
#define GD30BM2016_DM_OTINT_THRESHOLD                    0x8869U
#define GD30BM2016_DM_OTINT_DELAY                        0x886AU
#define GD30BM2016_DM_OTINT_RECOVERY_THRESHOLD           0x886BU
#define GD30BM2016_DM_UTC_THRESHOLD                      0x886CU
#define GD30BM2016_DM_UTC_DELAY                          0x886DU
#define GD30BM2016_DM_UTC_RECOVERY_THRESHOLD             0x886EU
#define GD30BM2016_DM_UTD_THRESHOLD                      0x886FU
#define GD30BM2016_DM_UTD_DELAY                          0x8870U
#define GD30BM2016_DM_UTD_RECOVERY_THRESHOLD             0x8871U
#define GD30BM2016_DM_UTINT_THRESHOLD                    0x8872U
#define GD30BM2016_DM_UTINT_DELAY                        0x8873U
#define GD30BM2016_DM_UTINT_RECOVERY_THRESHOLD           0x8874U
#define GD30BM2016_DM_RECOVERY_TIME                      0x8875U
#define GD30BM2016_DM_HWD_DELAY                          0x8876U
#define GD30BM2016_DM_PTO_CHARGE_THRESHOLD               0x8878U
#define GD30BM2016_DM_PTO_DELAY                          0x887AU
#define GD30BM2016_DM_PTO_RESET                          0x887CU
#define GD30BM2016_DM_CUDEP_THRESHOLD                    0x887EU
#define GD30BM2016_DM_CUDEP_DELAY                        0x8880U
#define GD30BM2016_DM_SUV_THRESHOLD                      0x8882U
#define GD30BM2016_DM_SUV_DELAY                          0x8884U
#define GD30BM2016_DM_SOV_THRESHOLD                      0x8886U
#define GD30BM2016_DM_SOV_DELAY                          0x8888U
#define GD30BM2016_DM_TOSS_THRESHOLD                     0x888AU
#define GD30BM2016_DM_TOSS_DELAY                         0x888CU
#define GD30BM2016_DM_SOCC_THRESHOLD                     0x888EU
#define GD30BM2016_DM_SOCC_DELAY                         0x8890U
#define GD30BM2016_DM_SOCD_THRESHOLD                     0x8892U
#define GD30BM2016_DM_SOCD_DELAY                         0x8894U
#define GD30BM2016_DM_SOT_THRESHOLD                      0x8895U
#define GD30BM2016_DM_SOT_DELAY                          0x8896U
#define GD30BM2016_DM_SOTF_THRESHOLD                     0x8897U
#define GD30BM2016_DM_SOTF_DELAY                         0x8898U
#define GD30BM2016_DM_VIMR_CHECK_VOLTAGE                 0x889AU
#define GD30BM2016_DM_VIMR_MAX_RELAX_CURRENT             0x889CU
#define GD30BM2016_DM_VIMR_THRESHOLD                     0x889EU
#define GD30BM2016_DM_VIMR_DELAY                         0x88A0U
#define GD30BM2016_DM_VIMR_RELAX_MIN_DURATION            0x88A2U
#define GD30BM2016_DM_VIMA_CHECK_VOLTAGE                 0x88A4U
#define GD30BM2016_DM_VIMA_MIN_ACTIVE_CURRENT            0x88A6U
#define GD30BM2016_DM_VIMA_THRESHOLD                     0x88A8U
#define GD30BM2016_DM_VIMA_DELAY                         0x88AAU
#define GD30BM2016_DM_CFETF_OFF_THRESHOLD                0x88ACU
#define GD30BM2016_DM_CFETF_OFF_DELAY                    0x88AEU
#define GD30BM2016_DM_DFETF_OFF_THRESHOLD                0x88B0U
#define GD30BM2016_DM_DFETF_OFF_DELAY                    0x88B2U
#define GD30BM2016_DM_VSSF_FAIL_THRESHOLD                0x88B4U
#define GD30BM2016_DM_VSSF_DELAY                         0x88B6U
#define GD30BM2016_DM_DUALLVL_DELAY                      0x88B7U
#define GD30BM2016_DM_LFOF_DELAY                         0x88B8U
#define GD30BM2016_DM_HWMX_DELAY                         0x88BAU
#define GD30BM2016_DM_SF_FAULT_MASK_A                    0x88BBU
#define GD30BM2016_DM_SF_FAULT_MASK_B                    0x88BCU
#define GD30BM2016_DM_SF_FAULT_MASK_C                    0x88BDU

/* 电芯均衡与负载检测寄存器。 */
#define GD30BM2016_DM_BALANCE_CONFIG                     0x8C00U
#define GD30BM2016_DM_BAL_MIN_CELL_TEMP                  0x8C01U
#define GD30BM2016_DM_BAL_MAX_CELL_TEMP                  0x8C02U
#define GD30BM2016_DM_BAL_MAX_INTERNAL_TEMP              0x8C03U
#define GD30BM2016_DM_BAL_CELL_INTERVAL                  0x8C04U
#define GD30BM2016_DM_BAL_MAX_CELLS                      0x8C05U
#define GD30BM2016_DM_BAL_MIN_CELL_V_CHARGE              0x8C06U
#define GD30BM2016_DM_BAL_MIN_DELTA_CHARGE               0x8C08U
#define GD30BM2016_DM_BAL_STOP_DELTA_CHARGE              0x8C09U
#define GD30BM2016_DM_BAL_MIN_CELL_V_RELAX               0x8C0AU
#define GD30BM2016_DM_BAL_MIN_DELTA_RELAX                0x8C0CU
#define GD30BM2016_DM_BAL_STOP_DELTA_RELAX               0x8C0DU
#define GD30BM2016_DM_LOAD_DETECT_ACTIVE_TIME            0x8C0EU
#define GD30BM2016_DM_LOAD_DETECT_RETRY_DELAY            0x8C0FU
#define GD30BM2016_DM_LOAD_DETECT_TIMEOUT                0x8C10U
#define GD30BM2016_DM_DSG_CURRENT_THRESHOLD              0x8C12U
#define GD30BM2016_DM_CHG_CURRENT_THRESHOLD              0x8C14U
#define GD30BM2016_DM_OPEN_WIRE_CHECK_TIME               0x8C16U

/* 常用位域定义。 */
#define GD30BM2016_CFETOFF_OPT_MASK                      0xFCU
#define GD30BM2016_CFETOFF_FXN_MASK                      0x03U
#define GD30BM2016_CFETOFF_DEFAULT                       0x10U
#define GD30BM2016_PIN_OPT_POLARITY_LOW                  0x80U
#define GD30BM2016_PIN_OPT_DIGITAL_HIZ                   0x40U
#define GD30BM2016_PIN_OPT_DRV_REG18                     0x20U
#define GD30BM2016_PIN_OPT_INPUT_ENABLE                  0x10U
#define GD30BM2016_PIN_OPT_PULLUP                        0x08U
#define GD30BM2016_PIN_OPT_PULLDOWN                      0x04U
#define GD30BM2016_PIN_FXN_UNUSED                        0x00U
#define GD30BM2016_PIN_FXN_GPO                           0x01U
#define GD30BM2016_PIN_FXN_STATUS                        0x02U
#define GD30BM2016_PIN_FXN_ADC_NTC                       0x03U

#define GD30BM2016_COMM_TYPE_I2C                         0x00U

#define GD30BM2016_FET_OPT_INIT_OFF                      0x20U
#define GD30BM2016_FET_OPT_PDSG_EN                       0x10U
#define GD30BM2016_FET_OPT_FET_CTRL_EN                   0x08U
#define GD30BM2016_FET_OPT_HOST_FET_EN                   0x04U
#define GD30BM2016_FET_OPT_SLEEP_CHG                     0x02U
#define GD30BM2016_FET_OPT_SERIES_FET                    0x01U
#define GD30BM2016_FET_OPTIONS_DEFAULT                   (GD30BM2016_FET_OPT_FET_CTRL_EN | \
                                                          GD30BM2016_FET_OPT_HOST_FET_EN | \
                                                          GD30BM2016_FET_OPT_SERIES_FET)
#define GD30BM2016_CHG_PUMP_SLEEP_CP_EN                  0x08U
#define GD30BM2016_CHG_PUMP_DSG_SFMODE_SLEEP             0x04U
#define GD30BM2016_CHG_PUMP_11V                          0x02U
#define GD30BM2016_CHG_PUMP_NORMAL_CP_EN                 0x01U

#define GD30BM2016_SAFETY_A_CUV                          (1U << 2)
#define GD30BM2016_SAFETY_A_COV                          (1U << 3)
#define GD30BM2016_SAFETY_A_OCC                          (1U << 4)
#define GD30BM2016_SAFETY_A_OCD1                         (1U << 5)
#define GD30BM2016_SAFETY_A_OCD2                         (1U << 6)
#define GD30BM2016_SAFETY_A_SCD                          (1U << 7)
#define GD30BM2016_SAFETY_A_CRITICAL_MASK                (GD30BM2016_SAFETY_A_CUV | \
                                                          GD30BM2016_SAFETY_A_COV | \
                                                          GD30BM2016_SAFETY_A_OCC | \
                                                          GD30BM2016_SAFETY_A_OCD1 | \
                                                          GD30BM2016_SAFETY_A_OCD2 | \
                                                          GD30BM2016_SAFETY_A_SCD)

#define GD30BM2016_SAFETY_B_OTC                          (1U << 4)
#define GD30BM2016_SAFETY_B_OTD                          (1U << 5)
#define GD30BM2016_SAFETY_B_OTF                          (1U << 6)
#define GD30BM2016_SAFETY_B_OTINT                        (1U << 7)
#define GD30BM2016_SAFETY_B_OVERTEMP_MASK                (GD30BM2016_SAFETY_B_OTC | \
                                                          GD30BM2016_SAFETY_B_OTD | \
                                                          GD30BM2016_SAFETY_B_OTF | \
                                                          GD30BM2016_SAFETY_B_OTINT)

#define GD30BM2016_SAFETY_C_OCD3                         (1U << 4)
#define GD30BM2016_SAFETY_C_SCDL                         (1U << 5)
#define GD30BM2016_SAFETY_C_OCDL                         (1U << 6)
#define GD30BM2016_SAFETY_C_COVL                         (1U << 7)
#define GD30BM2016_SAFETY_C_CRITICAL_MASK                (GD30BM2016_SAFETY_C_OCD3 | \
                                                          GD30BM2016_SAFETY_C_SCDL | \
                                                          GD30BM2016_SAFETY_C_OCDL | \
                                                          GD30BM2016_SAFETY_C_COVL)

#define GD30BM2016_BAL_OPW_EN                            0x80U
#define GD30BM2016_BAL_CB_EN                             0x40U
#define GD30BM2016_BAL_CB_ALERT_MASK                     0x20U
#define GD30BM2016_BAL_CB_NO_CMD                         0x10U
#define GD30BM2016_BAL_CB_NOSLEEP                        0x08U
#define GD30BM2016_BAL_CB_SLEEP                          0x04U
#define GD30BM2016_BAL_CB_RLX                            0x02U
#define GD30BM2016_BAL_CB_CHG                            0x01U
#define GD30BM2016_BAL_DEFAULT                           (GD30BM2016_BAL_OPW_EN | GD30BM2016_BAL_CB_EN)

#define GD30BM2016_POWER_CFG_BALANCE_MASK                0x000CU
#define GD30BM2016_POWER_CFG_BALANCE_SHIFT               2U
#define GD30BM2016_POWER_CFG_LFO_DEEPSLEEP               0x0200U
#define GD30BM2016_POWER_CFG_LDO_DEEPSLEEP               0x0400U

/* 首次调试阶段相对安全且有用的 DEMO 参数值。 */
#define GD30BM2016_DEMO_FET_OPTIONS                      0x0DU
#define GD30BM2016_DEMO_CHG_PUMP_CONTROL                 0x09U
#define GD30BM2016_DEMO_BALANCE_CONFIG                   0xC3U
#define GD30BM2016_DEMO_REG12_CONFIG                     0x11U
#define GD30BM2016_DEMO_REG0_CONFIG                      0x01U
#define GD30BM2016_DEMO_COMM_IDLE_TIME                   0x00U
#define GD30BM2016_DEMO_MANUFACTURING_FET_EN             0x50U

/* 单体均衡活动电芯可通过子命令数据写入。 */
#define GD30BM2016_BALANCE_REGISTER_VALID                1

#endif
