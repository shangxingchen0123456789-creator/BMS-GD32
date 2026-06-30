#ifndef BMS_BOARD_CONFIG_H
#define BMS_BOARD_CONFIG_H

#include "gd32g5x3.h"

/*
 * 板级资源映射，依据 2026-06-04 主控板/功率板网表和 BOM 整理。
 * 主控 BOM 为 GD32G553VET7，Keil 目标使用 GD32G553VE。
 * 硬件比例系数集中放在这里，后续标定时尽量不改动各个驱动文件。
 */

/*
 * Current BM2016 bring-up uses the verified I2C path.
 * The latest schematic net names may still say SPI_* on the AFE side, but
 * the final hardware/firmware communication mode for this board is I2C:
 * PB13/PB14 are bit-banged SCL/SDA, and PB12/PB15 are held low as
 * address strap pins for the tested 0x20/0x21 I2C address.
 */

#ifndef BMS_ENABLE_POWER_ADC
#define BMS_ENABLE_POWER_ADC                  1
#endif

#ifndef BMS_ENABLE_HRTIMER_PWM
#define BMS_ENABLE_HRTIMER_PWM                1
#endif

/*
 * 主控板 H2.2/H2.1：MCU_UART_TX/RX -> PC10/PC11。
 * GD32G553_power 工程已验证该连线应使用 USART2 + AF7，主工程同步采用同一映射，
 * 避免上位机连接后收不到下位机主动上报帧。
 */
#define BMS_UART_PERIPH                       USART2
#define BMS_UART_CLK                          RCU_USART2
#define BMS_UART_IRQn                         USART2_IRQn
#define BMS_UART_GPIO_CLK                     RCU_GPIOC
#define BMS_UART_GPIO_PORT                    GPIOC
#define BMS_UART_TX_PIN                       GPIO_PIN_10
#define BMS_UART_RX_PIN                       GPIO_PIN_11
#define BMS_UART_AF                           GPIO_AF_7
#define BMS_UART_BAUDRATE                     115200U

/* BM2016 I2C bus pins. */
#define BMS_AFE_I2C_GPIO_CLK                  RCU_GPIOB
#define BMS_AFE_I2C_GPIO_PORT                 GPIOB
#define BMS_AFE_I2C_ADDR0_PIN                 GPIO_PIN_12
#define BMS_AFE_I2C_SCL_PIN                   GPIO_PIN_13
#define BMS_AFE_I2C_SDA_PIN                   GPIO_PIN_14
#define BMS_AFE_I2C_ADDR1_PIN                 GPIO_PIN_15

/* BM2016 sideband GPIOs confirmed from mainboard schematic. */
#define BMS_AFE_ALERT_GPIO_CLK                RCU_GPIOD
#define BMS_AFE_ALERT_GPIO_PORT               GPIOD
#define BMS_AFE_ALERT_PIN                     GPIO_PIN_10
#define BMS_AFE_RST_GPIO_CLK                  RCU_GPIOC
#define BMS_AFE_RST_GPIO_PORT                 GPIOC
#define BMS_AFE_RST_PIN                       GPIO_PIN_4
#define BMS_AFE_DDSG_DCHG_GPIO_CLK            RCU_GPIOB
#define BMS_AFE_DDSG_DCHG_GPIO_PORT           GPIOB
#define BMS_AFE_DDSG_PIN                      GPIO_PIN_0
#define BMS_AFE_DCHG_PIN                      GPIO_PIN_1
#define BMS_AFE_DFETOFF_GPIO_CLK              RCU_GPIOB
#define BMS_AFE_DFETOFF_GPIO_PORT             GPIOB
#define BMS_AFE_DFETOFF_PIN                   GPIO_PIN_2
#define BMS_AFE_ALERT_EXTI_LINE               EXTI_10
#define BMS_AFE_ALERT_EXTI_PORT_SOURCE        EXTI_SOURCE_GPIOD
#define BMS_AFE_ALERT_EXTI_PIN_SOURCE         EXTI_SOURCE_PIN10

/*
 * Optional FAN_PWM hardware is on PB6. Current firmware does not drive the fan
 * automatically, so the pin is only reserved here for future control logic.
 */
#define BMS_FAN_PWM_GPIO_CLK                  RCU_GPIOB
#define BMS_FAN_PWM_GPIO_PORT                 GPIOB
#define BMS_FAN_PWM_PIN                       GPIO_PIN_6

/*
 * Bring-up board does not populate a real fuse/permanent-failure action path.
 * Keep cell voltage/current/temperature protections active, but do not latch
 * BM2016 PF/FUSE history bits as system faults until that hardware path is
 * deliberately validated.
 */
#ifndef BMS_ENABLE_AFE_PERMANENT_FAULT_MONITOR
#define BMS_ENABLE_AFE_PERMANENT_FAULT_MONITOR 0
#endif

#ifndef BMS_ENABLE_AFE_FUSE_FLAG_MONITOR
#define BMS_ENABLE_AFE_FUSE_FLAG_MONITOR      0
#endif

#ifndef BMS_ENABLE_INPUT_UV_FAULT
#define BMS_ENABLE_INPUT_UV_FAULT             0
#endif

/* 主控板新网表中 FAULT_OC 由 LMV331IDBVR 比较器输出到 U1.73，即 PA12。 */
#ifndef BMS_ENABLE_POWER_FAULT_PIN
#define BMS_ENABLE_POWER_FAULT_PIN            0
#endif
#ifndef BMS_POWER_FAULT_PIN_FAST_LATCH_ENABLE
#define BMS_POWER_FAULT_PIN_FAST_LATCH_ENABLE 0U
#endif

/*
 * 功率板 ADC 输入通道。
 *
 * 主控板 H5 与功率板 H2 一一相连。注意 GD32G553VET7 的 ADC 通道
 * 不能按 PAx = ADCx_INx 推断，PA4/PA5 尤其容易配错：
 * - H5.1 / H2.1   ADC_Iin      -> PA0 / ADC0_IN0；
 * - H5.2 / H2.2   ADC_Vin      -> PA1 / ADC0_IN1；
 * - H5.3 / H2.3   ADC_Vout     -> PA2 / ADC0_IN2；
 * - H5.4 / H2.4   ADC_Iout     -> PA3 / ADC0_IN3；
 * - H5.5 / H2.5   ADC_MOS_TEMP -> PA4 / ADC1_IN15；
 * - H5.6 / H2.6   ADC_L_TEMP   -> PA5 / ADC1_IN12。
 *
 * H5.8/PWR_EN does not drive any power-board enable circuit in the latest
 * hardware, so firmware intentionally does not control a PWR_EN GPIO.
 */
#define BMS_ADC_IIN_GPIO_CLK                  RCU_GPIOA
#define BMS_ADC_IIN_GPIO_PORT                 GPIOA
#define BMS_ADC_IIN_PIN                       GPIO_PIN_0
#define BMS_ADC_IIN_PERIPH                    ADC0
#define BMS_ADC_IIN_CHANNEL                   ADC_CHANNEL_0
#define BMS_ADC_VIN_GPIO_CLK                  RCU_GPIOA
#define BMS_ADC_VIN_GPIO_PORT                 GPIOA
#define BMS_ADC_VIN_PIN                       GPIO_PIN_1
#define BMS_ADC_VIN_PERIPH                    ADC0
#define BMS_ADC_VIN_CHANNEL                   ADC_CHANNEL_1
#define BMS_ADC_VOUT_GPIO_CLK                 RCU_GPIOA
#define BMS_ADC_VOUT_GPIO_PORT                GPIOA
#define BMS_ADC_VOUT_PIN                      GPIO_PIN_2
#define BMS_ADC_VOUT_PERIPH                   ADC0
#define BMS_ADC_VOUT_CHANNEL                  ADC_CHANNEL_2
#define BMS_ADC_IOUT_GPIO_CLK                 RCU_GPIOA
#define BMS_ADC_IOUT_GPIO_PORT                GPIOA
#define BMS_ADC_IOUT_PIN                      GPIO_PIN_3
#define BMS_ADC_IOUT_PERIPH                   ADC0
#define BMS_ADC_IOUT_CHANNEL                  ADC_CHANNEL_3
#define BMS_ADC_MOS_TEMP_GPIO_CLK             RCU_GPIOA
#define BMS_ADC_MOS_TEMP_GPIO_PORT            GPIOA
#define BMS_ADC_MOS_TEMP_PIN                  GPIO_PIN_4
#define BMS_ADC_MOS_TEMP_PERIPH               ADC1
#define BMS_ADC_MOS_TEMP_CHANNEL              ADC_CHANNEL_15
#define BMS_ADC_L_TEMP_GPIO_CLK               RCU_GPIOA
#define BMS_ADC_L_TEMP_GPIO_PORT              GPIOA
#define BMS_ADC_L_TEMP_PIN                    GPIO_PIN_5
#define BMS_ADC_L_TEMP_PERIPH                 ADC1
#define BMS_ADC_L_TEMP_CHANNEL                ADC_CHANNEL_12

#ifndef BMS_BOARD_STATIC_ASSERT
#define BMS_BOARD_STATIC_ASSERT(name, expr)   typedef char bms_board_static_assert_##name[(expr) ? 1 : -1]
#endif

BMS_BOARD_STATIC_ASSERT(adc_iin_map,
                        BMS_ADC_IIN_GPIO_CLK == RCU_GPIOA &&
                        BMS_ADC_IIN_GPIO_PORT == GPIOA &&
                        BMS_ADC_IIN_PIN == GPIO_PIN_0 &&
                        BMS_ADC_IIN_PERIPH == ADC0 &&
                        BMS_ADC_IIN_CHANNEL == ADC_CHANNEL_0);
BMS_BOARD_STATIC_ASSERT(adc_vin_map,
                        BMS_ADC_VIN_GPIO_CLK == RCU_GPIOA &&
                        BMS_ADC_VIN_GPIO_PORT == GPIOA &&
                        BMS_ADC_VIN_PIN == GPIO_PIN_1 &&
                        BMS_ADC_VIN_PERIPH == ADC0 &&
                        BMS_ADC_VIN_CHANNEL == ADC_CHANNEL_1);
BMS_BOARD_STATIC_ASSERT(adc_vout_map,
                        BMS_ADC_VOUT_GPIO_CLK == RCU_GPIOA &&
                        BMS_ADC_VOUT_GPIO_PORT == GPIOA &&
                        BMS_ADC_VOUT_PIN == GPIO_PIN_2 &&
                        BMS_ADC_VOUT_PERIPH == ADC0 &&
                        BMS_ADC_VOUT_CHANNEL == ADC_CHANNEL_2);
BMS_BOARD_STATIC_ASSERT(adc_iout_map,
                        BMS_ADC_IOUT_GPIO_CLK == RCU_GPIOA &&
                        BMS_ADC_IOUT_GPIO_PORT == GPIOA &&
                        BMS_ADC_IOUT_PIN == GPIO_PIN_3 &&
                        BMS_ADC_IOUT_PERIPH == ADC0 &&
                        BMS_ADC_IOUT_CHANNEL == ADC_CHANNEL_3);
BMS_BOARD_STATIC_ASSERT(adc_mos_temp_map,
                        BMS_ADC_MOS_TEMP_GPIO_CLK == RCU_GPIOA &&
                        BMS_ADC_MOS_TEMP_GPIO_PORT == GPIOA &&
                        BMS_ADC_MOS_TEMP_PIN == GPIO_PIN_4 &&
                        BMS_ADC_MOS_TEMP_PERIPH == ADC1 &&
                        BMS_ADC_MOS_TEMP_CHANNEL == ADC_CHANNEL_15);
BMS_BOARD_STATIC_ASSERT(adc_l_temp_map,
                        BMS_ADC_L_TEMP_GPIO_CLK == RCU_GPIOA &&
                        BMS_ADC_L_TEMP_GPIO_PORT == GPIOA &&
                        BMS_ADC_L_TEMP_PIN == GPIO_PIN_5 &&
                        BMS_ADC_L_TEMP_PERIPH == ADC1 &&
                        BMS_ADC_L_TEMP_CHANNEL == ADC_CHANNEL_12);

#define BMS_ADC_VREF_MV                       3300U
#define BMS_ADC_MAX_RAW                       4095U
#define BMS_ADC_SAMPLE_TIME                   239U
#ifndef BMS_ENABLE_PWM_SYNC_ADC
#define BMS_ENABLE_PWM_SYNC_ADC               1
#endif
/*
 * Fixed-phase current sampling is useful for low-noise bench checks, but Boost
 * output current is pulse-like and the 80% phase can fall in the low-side
 * on-time. adc_manager.c therefore uses asynchronous averaging for Iout while
 * Boost is active, and keeps this phase as the fallback synchronized sample
 * point for non-Boost current reads.
 */
#define BMS_ADC_SYNC_PHASE_X10000             8000U
#define BMS_ADC_SYNC_WAIT_TIMEOUT             2000U
#define BMS_ADC_CURRENT_SAMPLE_TIME           55U
#define BMS_ADC_FAST_VOLTAGE_AVERAGE_SAMPLES  10U
#define BMS_ADC_FAST_CURRENT_AVERAGE_SAMPLES  10U
#define BMS_ADC_TIMEOUT                       20000U

/*
 * ADC 标定系数来自已跑通的 GD32G553_power 工程。
 *
 * 电压：引脚电压 mV * gain_x1000 / 1000 = 实际 Vin/Vout mV。
 * 电流：功率板使用 5mR 采样电阻和约 62 倍放大，1A 约对应 ADC 引脚 0.31V，
 *       因此 1mV 约为 3.226mA，这里用 x1000 保留小数精度。
 */
#define BMS_ADC_VIN_GAIN_X1000                16000U
#define BMS_ADC_VOUT_GAIN_X1000               16000U
#define BMS_ADC_IIN_OFFSET_RAW                7U
#define BMS_ADC_IOUT_OFFSET_RAW               6U
/* Current calibration constants from the 12 V electronic-load CC bench run. */
#define BMS_ADC_IIN_MA_PER_MV_X1000           1650U
#define BMS_ADC_IOUT_MA_PER_MV_X1000          2600U
/*
 * MOS/L temperature divider: P3V3 -> 10k/B3950 NTC -> ADC -> 10k -> GND.
 * The ADC voltage rises as temperature rises, so adc_manager.c uses an NTC
 * lookup table instead of the old linear mV-per-degree approximation.
 */
#define BMS_ADC_TEMP_NTC_R25_OHM              10000U
#define BMS_ADC_TEMP_PULLDOWN_OHM             10000U
#define BMS_ADC_TEMP_NTC_BETA                 3950U

/* System protection and algorithm defaults. */
#define BMS_DEFAULT_TRICKLE_CURRENT_MA        200U
#define BMS_DEFAULT_INPUT_UV_THRESHOLD_MV     10000U
#define BMS_DEFAULT_OUTPUT_OVP_MARGIN_MV      300U
#define BMS_DEFAULT_OUTPUT_OCP_MARGIN_MA      1500U
#define BMS_DEFAULT_INPUT_CURRENT_LIMIT_MA    6500U
#define BMS_DEFAULT_INPUT_POWER_LIMIT_W       260U
#define BMS_DEFAULT_MIN_CHARGE_CURRENT_MA     100U
#define BMS_DEFAULT_DERATE_START_TEMP_X10     500
#define BMS_DEFAULT_DERATE_STOP_TEMP_X10      600
#define BMS_DEFAULT_SOC_CAPACITY_MAH          5000U
#define BMS_DEFAULT_CURRENT_KP_DIV            80U
#define BMS_DEFAULT_CURRENT_KI_DIV            8000U
#define BMS_DEFAULT_VOLTAGE_KP_DIV            100U
#define BMS_DEFAULT_VOLTAGE_KI_DIV            10000U
#define BMS_DEFAULT_LOOP_STEP_MAX_X100        80U

/* Manual digital-power mode limits. The mode still uses the same ADC, PWM and
 * safety lockout path as the charger state machine. */
#define BMS_DIGITAL_POWER_MIN_OUTPUT_MV       5000U
#define BMS_DIGITAL_POWER_MAX_OUTPUT_MV       42000U
#define BMS_DIGITAL_POWER_MIN_CURRENT_MA      100U
#define BMS_DIGITAL_POWER_MAX_CURRENT_MA      10000U
/*
 * Digital-power bench mode can momentarily overshoot while crossing the
 * Buck/Buck-Boost/Boost boundary. Keep a hard runaway limit, but require the
 * normal target+margin OVP to persist before latching a fault.
 */
#define BMS_DIGITAL_POWER_OUTPUT_OVP_MARGIN_MV 2000U
#ifndef BMS_BATTERY_FULL_CHARGE_VOLTAGE_MV
#define BMS_BATTERY_FULL_CHARGE_VOLTAGE_MV 37800U
#endif
#define BMS_PRECONNECT_OUTPUT_OVP_MARGIN_MV  2000U
#define BMS_PRECONNECT_OUTPUT_OVP_HARD_MARGIN_MV 5000U
#define BMS_OUTPUT_OVP_HARD_MARGIN_MV         5000U
#define BMS_OUTPUT_OVP_CONFIRM_SAMPLES        8U
#define BMS_OUTPUT_OVP_STARTUP_BLANK_SAMPLES  80U
#define BMS_OUTPUT_OVP_CONFIRM_CONTROL_COUNT  2U

/*
 * Bench-only path for testing the buck-boost power board without BM2016/pack.
 * It ignores AFE/cell/path faults only in manual digital-power bring-up; power
 * ADC, PWM, FAULT_OC, Vout OVP, OCP, OTP and emergency-stop protections remain.
 */
#ifndef BMS_DIGITAL_POWER_AFELESS_DEBUG
#define BMS_DIGITAL_POWER_AFELESS_DEBUG       1
#endif

/*
 * GD32G553VE project target reserves the last 16 KB of the 512 KB main flash
 * for non-volatile parameters and fault history.
 */
#define BMS_STORAGE_FLASH_BASE_ADDRESS        0x0807C000UL
#define BMS_STORAGE_FLASH_SIZE_BYTES          0x00004000UL
#define BMS_PARAM_STORAGE_BASE_ADDRESS        BMS_STORAGE_FLASH_BASE_ADDRESS
#define BMS_PARAM_STORAGE_SIZE_BYTES          0x00001000UL
#define BMS_FAULT_LOG_BASE_ADDRESS            (BMS_STORAGE_FLASH_BASE_ADDRESS + BMS_PARAM_STORAGE_SIZE_BYTES)
#define BMS_FAULT_LOG_SIZE_BYTES              (BMS_STORAGE_FLASH_SIZE_BYTES - BMS_PARAM_STORAGE_SIZE_BYTES)

/*
 * PA8/PA9/PA10/PA11：PWM1H/1L/2H/2L；PA12：FAULT_OC。
 * 新网表中主控板 H4 与功率板 H1 一一相连：
 * - H4.4 / H1.4 PWM1H；
 * - H4.3 / H1.3 PWM1L；
 * - H4.2 / H1.2 PWM2H；
 * - H4.1 / H1.1 PWM2L。
 */
#define BMS_HRTIMER_PERIPH                    HRTIMER0
#define BMS_HRTIMER_CLK                       RCU_HRTIMER
#define BMS_PWM_GPIO_CLK                      RCU_GPIOA
#define BMS_PWM_GPIO_PORT                     GPIOA
#define BMS_PWM1H_PIN                         GPIO_PIN_8
#define BMS_PWM1L_PIN                         GPIO_PIN_9
#define BMS_PWM2H_PIN                         GPIO_PIN_10
#define BMS_PWM2L_PIN                         GPIO_PIN_11
#define BMS_PWM_FAULT_PIN                     GPIO_PIN_12
#define BMS_PWM_FAULT_EXTI_LINE               EXTI_12
#define BMS_PWM_FAULT_EXTI_PORT_SOURCE        EXTI_SOURCE_GPIOA
#define BMS_PWM_FAULT_EXTI_PIN_SOURCE         EXTI_SOURCE_PIN12
/* GD32G553_power 已验证 PA8~PA11 HRTIMER_ST0/ST1 输出使用 AF13。 */
#define BMS_PWM_AF                            GPIO_AF_13
#define BMS_PWM_FREQUENCY_HZ                  200000U
/* EG3112 has 50..300 ns internal deadtime; keep MCU non-overlap short enough for 95% duty at 200 kHz. */
#define BMS_PWM_DEADTIME_NS                   120U
#define BMS_PWM_DUTY_MIN_X100                 200U
#define BMS_PWM_DUTY_MAX_X100                 9500U
#define BMS_PWM_OUTPUT_CHANNELS               (HRTIMER_ST0_CH0 | HRTIMER_ST0_CH1 | HRTIMER_ST1_CH0 | HRTIMER_ST1_CH1)
#define BMS_PWM_COUNTERS                      (HRTIMER_ST0_COUNTER | HRTIMER_ST1_COUNTER)

#define BMS_POWER_EXTERNAL_PRESENT_MV         8000U
#define BMS_POWER_EXTERNAL_HYSTERESIS_MV      1000U
#define BMS_POWER_INPUT_PRESENT_MAX_MV        60000U
#ifndef BMS_POWER_PRECONNECT_ASYNC_BOOST_RECTIFIER
#define BMS_POWER_PRECONNECT_ASYNC_BOOST_RECTIFIER 1U
#endif
#ifndef BMS_POWER_LIGHT_LOAD_ASYNC_BOOST_RECTIFIER
#define BMS_POWER_LIGHT_LOAD_ASYNC_BOOST_RECTIFIER 1U
#endif
#ifndef BMS_POWER_BATTERY_BOOST_DUTY_LIMIT_ENABLE
#define BMS_POWER_BATTERY_BOOST_DUTY_LIMIT_ENABLE 1U
#endif
#ifndef BMS_POWER_BATTERY_BOOST_DUTY_HEADROOM_X100
/*
 * Handover 起始 duty headroom 从 600（6%）降到 0。
 * 原 6% headroom 让 handover 起始 duty 过高，导致 Vout 冲破硬 OVP。
 * 理想 duty = (target-vin)/target 对应的升压比已经够用，无需额外余量。
 */
#define BMS_POWER_BATTERY_BOOST_DUTY_HEADROOM_X100 0U
#endif
#ifndef BMS_POWER_BATTERY_BOOST_DUTY_MAX_X100
#define BMS_POWER_BATTERY_BOOST_DUTY_MAX_X100 5200U
#endif
#ifndef BMS_POWER_AFE_HANDOVER_GUARD_CYCLES
#define BMS_POWER_AFE_HANDOVER_GUARD_CYCLES 50U
#endif
#ifndef BMS_POWER_AFE_HANDOVER_BOOST_DUTY_MAX_X100
#define BMS_POWER_AFE_HANDOVER_BOOST_DUTY_MAX_X100 4500U
#endif
/*
 * Bench bring-up guard: with the power-board ADC harness disconnected, VIN can
 * float near raw 1300/4095, which scales to about 16 V. During the actual 24 V
 * charger joint-debug, this window can misclassify a noisy/sagging real VIN as
 * absent input and create a boost/drop/retry loop, so keep it disabled here.
 * Re-enable only for disconnected-harness bench bring-up.
 */
#define BMS_POWER_FLOATING_VIN_REJECT_ENABLE  0U
#define BMS_POWER_FLOATING_VIN_MIN_MV         14000U
#define BMS_POWER_FLOATING_VIN_MAX_MV         18000U
#define BMS_POWER_FULL_CELL_MARGIN_MV         20U

/*
 * Before the BM2016 main CHG/DSG FETs are closed, use the board-level
 * precharge/predischarge path as the current-limited bridge. The PWM power
 * stage may add a small preconnect boost current when the resistor path alone
 * leaves Vout below the pack closing window.
 */
/*
 * 预充路径配置：PCHG + DSG（不是 PDSG）
 * 硬件拓扑：VIN+ → 187.5Ω → PCHG → bat+ → 电池 → bat- → DSG → VIN-
 * 预充需要预充电阻(PCHG路径)限流 + 主放电FET(DSG)提供回路。
 * PDSG 是预放电用，不参与预充。
 */
#define BMS_PATH_HARDWARE_PRECONNECT_ENABLE   1U
#define BMS_PATH_HARDWARE_PRECONNECT_USE_PCHG 1U
#define BMS_PATH_HARDWARE_PRECONNECT_USE_PDSG 0U  // 改为 0，不用 PDSG
#define BMS_PATH_HARDWARE_PRECONNECT_USE_DSG  1U  // 新增，预充需要 DSG 回路
#define BMS_MANUAL_PRECONNECT_FET_ALLOW_WITH_FAULTS 1U
#define BMS_PATH_PRECONNECT_TARGET_DELTA_MV   50U
#define BMS_PATH_PRECONNECT_DELTA_MV          50U
#define BMS_PATH_PRECONNECT_LOW_DELTA_MV      50U
#define BMS_PATH_PRECONNECT_BULK_DELTA_MV     3000U
#define BMS_PATH_PRECONNECT_CONFIRM_COUNT     1U
#define BMS_PATH_PRECONNECT_TARGET_ABOVE_PACK_MV 20U
/*
 * 预连接电流从 150mA 降到 100mA，进一步减缓充电速度。
 * 轻载下即使 150mA 仍会导致 Vout 过冲 500~600mV 超芯片门闩。
 * 降到 100mA 让 boost 更慢爬升，配合 1000mV coast 阈值控制过冲。
 */
#define BMS_PATH_PRECONNECT_CURRENT_MA        100U
#define BMS_PATH_PRECONNECT_HOLD_CURRENT_MA   150U
#define BMS_PATH_PRECONNECT_MID_CURRENT_MA    300U
#define BMS_PATH_PRECONNECT_TARGET_STEP_MV    500U
#define BMS_PATH_PRECONNECT_BOOST_MAX_MS      0U
#define BMS_PATH_PRECONNECT_BOOST_COOLDOWN_MS 0U
/* Debug only: bypass Vout-Stack preconnect gating and close BM2016 CHG+DSG directly. */
#define BMS_PATH_BYPASS_PRECONNECT_FOR_TEST   0U

#endif
