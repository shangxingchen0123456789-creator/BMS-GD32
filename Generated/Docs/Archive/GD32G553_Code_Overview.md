# GD32G553 主工程代码梳理与当前实现说明

更新时间：2026-06-30  
工程路径：`GD32G553/GD32G553`  
上位机路径：`BMSMonitorSrc`  
参考工程：`GD32G553_power`

## 1. 当前结论

当前主工程已经按竞赛功能整理为正式运行版本：

1. 固件不再生成模拟电芯电压、模拟 ADC、电流或温度数据。
2. BM2016 AFE 读取失败时，只上报 `BMS_FAULT_AFE_COMM`，电芯数据保持 0。
3. 功率板 ADC 读取失败时，只上报 `BMS_FAULT_ADC`，功率采样数据保持 0。
4. `board_test.c/.h` 已从源码和 Keil 工程中删除，测试入口不再混入正式固件。
5. 上位机本地演示遥测入口已移除，只显示下位机真实串口帧。
6. 通信服务初始化前会先采集一次真实状态；通信任务第一次轮询立即发送实时状态和单体电压。
7. 测试电源板已验证 Buck、Boost 和 ADC 正常，主工程已同步采用测试工程中跑通的 PWM、ADC 和串口映射。

历史编译结果：

- 固件 Keil：`0 Error(s), 0 Warning(s)`，日志 `GD32G553/GD32G553/Project/codex_build_production.log`。
- 上位机 Qt/CMake：`[100%] Built target BMSMonitor`。

## 2. 硬件与板级配置

板级配置集中在 `Config/bms_board_config.h`，其它驱动不再散落硬件比例或引脚映射。

### 2.1 上位机串口

主控板 H3 使用：

- `PC10`：MCU_UART_TX
- `PC11`：MCU_UART_RX
- 外设：`USART2`
- 复用：`GPIO_AF_7`
- 波特率：`115200`
- 中断：`USART2_IRQn` / `USART2_IRQHandler`

该映射来自 `GD32G553_power` 测试工程，并已替换主工程原先的 `UART3/AF8`。

### 2.2 AFE

GD30BM2016 当前固件使用已验证的 I2C 通信路径：

- `PB13`：SCL
- `PB14`：SDA
- `PB12/PB15`：地址 strap，引脚保持低电平以匹配已验证的 `0x20/0x21` I2C 地址。
- I2C 实现：软件模拟时序。

侧带安全信号：

- `PD10`：AFE ALERT，低有效，通过 EXTI10 进入最高优先级故障路径。
- `PC4`：RST_SHUT。
- `PB0/PB1`：DDSG/DCHG AFE 状态输入。
- `PB2`：DFETOFF，故障时 MCU 直接拉高，要求 BM2016 立即切断放电路径。

### 2.3 功率板 ADC

ADC 通道按主控板/功率板网表和 `GD32G553VET7` 数据手册确认。这里不能按
`PAx = ADCx_INx` 直接推断，`PA4/PA5` 必须走 `ADC1` 的非顺序通道：

| 信号 | MCU 引脚 | ADC |
|---|---:|---|
| ADC_Iin | PA0 | ADC0 CH0 |
| ADC_Vin | PA1 | ADC0 CH1 |
| ADC_Vout | PA2 | ADC0 CH2 |
| ADC_Iout | PA3 | ADC0 CH3 |
| ADC_MOS_TEMP | PA4 | ADC1 CH15 |
| ADC_L_TEMP | PA5 | ADC1 CH12 |

`bms_board_config.h` 中已经对这 6 路 ADC 的 GPIO、ADC 外设和通道做了编译期
静态检查；如果后续误把 `ADC_MOS_TEMP` 改成 `ADC0 CH4` 或把 `ADC_L_TEMP`
改成 `ADC0 CH5`，工程会直接报错，避免上板后才发现 raw 码来自错误通道。

标定参数也已迁回主工程：

- `BMS_ADC_VIN_GAIN_X1000 = 15470`
- `BMS_ADC_VOUT_GAIN_X1000 = 16400`
- `BMS_ADC_IIN_OFFSET_RAW = 7`
- `BMS_ADC_IOUT_OFFSET_RAW = 6`
- `BMS_ADC_IIN_MA_PER_MV_X1000 = 2160`
- `BMS_ADC_IOUT_MA_PER_MV_X1000 = 1650`

电流系数含义：功率板 5mR 采样电阻、约 62 倍放大，约 `1A -> 0.31V`，因此 `1mV -> 3.226mA`。

### 2.4 HRTIMER PWM

四路 PWM：

- `PA8`：PWM1H
- `PA9`：PWM1L
- `PA10`：PWM2H
- `PA11`：PWM2L
- 复用：`GPIO_AF_13`
- 频率：`200 kHz`
- 死区：`120 ns`

`PA12` 为 `FAULT_OC`，高有效。触发后：

1. HRTIMER fault 硬件先把 PWM 输出拉到 inactive。
2. EXTI12 进入 `EXTI10_15_IRQHandler()`。
3. ISR 立刻调用 `safety_manager_handle_external_fault_isr()`。
4. 软件同步关闭 PWM，并拉高 BM2016 DFETOFF。

## 3. 顶层启动流程

入口文件是 `User/main.c`，顶层流程为：

1. `system_clock_config()`：系统时钟初始化。
2. `systick_config()`：SysTick 初始化。
3. `board_support_init()`：LED 等板级辅助初始化。
4. `app_tasks_start()`：初始化 BMS 驱动、管理模块和 FreeRTOS 任务。
5. `vTaskStartScheduler()`：启动调度器。

`App/app_tasks.c` 是系统调度核心。初始化顺序为：

1. `bms_state_init()`：共享状态先置为“无有效采样 + AFE/ADC 故障”。
2. `afe_gd30bm2016_init()`：BM2016 唤醒、SPI 切换、保护参数写入。
3. `adc_manager_init()`：ADC0/ADC1 初始化与校准。
4. `power_control_init()`：HRTIMER PWM 初始化，但不打开输出。
5. `safety_manager_init()`：AFE ALERT 和 FAULT_OC 外部中断初始化。
6. `power_path_manager_init()`：默认强制关闭 Q4/Q5。
7. `charge_manager_init()`：充电参数、状态机和故障锁存初始化。
8. `balance_manager_init()`：均衡位图清零。
9. `soc_estimator_init()`：SOC 等待真实电芯电压初始化。
10. `app_collect_and_publish_status(0)`：先采集一次真实 AFE/ADC 并发布状态。
11. `bms_comm_service_init()`：串口和协议服务初始化。
12. 创建 FreeRTOS 任务。

第 10 步是为了保证上位机一连接后，第一帧就是真实采样结果；如果硬件异常，则第一帧明确带故障位。

## 4. FreeRTOS 任务优先级

`FreeRTOSConfig.h` 中 `configMAX_PRIORITIES = 6`。任务优先级设计如下：

| 任务 | 周期 | 优先级 | 说明 |
|---|---:|---:|---|
| `bms_safety` | 2 ms | 最高 | 快速检查 ALERT/FAULT_OC，故障时立即关断 Q4/Q5 |
| `bms_comm` | 10 ms | 次高 | 非阻塞 UART 收发，上位机实时通信 |
| `bms_ctrl` | 20 ms | 中 | AFE/ADC 采样、状态机、SOC、均衡、功率目标 |
| `heartbeat` | 500 ms | 低 | LED 心跳 |

通信任务优先级高于控制任务，且 UART RX/TX 都使用中断环形缓冲，避免串口接收因任务周期过慢丢字节，也避免发送阻塞控制循环。

## 5. 真实采样链路

### 5.1 AFE：`afe_gd30bm2016.c`

对外接口：

- `afe_gd30bm2016_init()`
- `afe_gd30bm2016_poll()`
- `afe_gd30bm2016_set_balance()`
- `afe_gd30bm2016_force_path_off_fast()`
- `afe_gd30bm2016_fets_off()`
- `afe_gd30bm2016_fets_on()`
- `afe_gd30bm2016_alert_active()`

当前策略：

1. 只读取 BM2016 真实寄存器。
2. 读取 9 串单体电压，范围检查为 `500mV ~ 5000mV`。
3. 读取 PACK 引脚电压。
4. 读取 4 路温度。
5. 读取 Safety/PF/FUSE 状态并映射到固件故障位。
6. 若任何关键通信失败，本轮只置 `BMS_FAULT_AFE_COMM`，不填默认电芯电压。

AFE 保护映射：

- COV/COVL -> `BMS_FAULT_CELL_OVP`
- CUV -> `BMS_FAULT_CELL_UVP`
- OCC/OCD -> `BMS_FAULT_CHARGE_OCP`
- SCD/SCDL -> `BMS_FAULT_SHORT_CIRCUIT`
- 过温 -> `BMS_FAULT_OTP`
- PF 任意位 -> `BMS_FAULT_AFE_PROTECTION`
- FUSE flag -> `BMS_FAULT_FUSE`

BM2016 初始化中也配置了：

- 9 串模式；
- 单体过压/欠压阈值；
- OCC/OCD/SCD 保护；
- CHG/DSG FET 保护掩码；
- 预充、预放相关参数；
- 被动均衡参数。

### 5.2 功率 ADC：`adc_manager.c`

对外接口：

- `adc_manager_init()`
- `adc_manager_sample()`

采样顺序：

1. IIN
2. VIN
3. VOUT
4. IOUT
5. MOS 温度
6. 电感温度

当前策略：

- 任一路 ADC 超时或失败，本轮 `bms_power_sample_t` 保持全 0，并置 `BMS_FAULT_ADC`。
- 不再使用输出电压缓慢逼近、输入电压固定 46V、温度缓慢变化等软件造数。
- `faultOcActive` 直接读取 PA12。

## 6. 故障最高优先级设计

故障处理核心文件：

- `safety_manager.c`
- `gd32g5x3_it.c`
- `charge_manager.c`
- `power_path_manager.c`
- `afe_gd30bm2016.c`
- `power_control.c`

故障路径分两层：

### 6.1 中断快速关断

`EXTI10_15_IRQHandler()` 处理：

- AFE ALERT：映射 `BMS_FAULT_AFE_PROTECTION`
- FAULT_OC：映射 `BMS_FAULT_CHARGE_OCP`

ISR 内只做最快动作：

1. 锁存故障位。
2. `power_control_stop()` 关闭 HRTIMER PWM 输出。
3. `afe_gd30bm2016_force_path_off_fast()` 拉高 DFETOFF。

ISR 不做 SPI 访问，避免中断中阻塞。

### 6.2 最高优先级安全任务

`bms_safety` 每 2 ms 调用 `safety_manager_service()`：

1. 轮询 ALERT/FAULT_OC 当前电平。
2. 若有故障，调用 `safety_manager_report_faults()`。
3. 在任务上下文补发 BM2016 FET 子命令：
   - `CHG_PCHG_OFF`
   - `DSG_PDSG_OFF`
   - `ALL_FETS_OFF`

这样可以同时覆盖充电、放电、预充、预放路径。

## 7. 电源路径管理规则

文件：`power_path_manager.c`

设计目标：

- 正常无故障时，根据外部电源和电池充满状态决定 Q4/Q5。
- 任意故障时强制关闭 Q4/Q5。
- 故障未清除前禁止自动恢复。

规则：

1. `fault_bitmap != 0`：强制 `power_path_manager_force_off()`。
2. ADC 无采样对象：强制关断。
3. 外部电源存在并且电池已满：关闭电池路径，由外部电源供电。
4. 无外部电源，或电池未满：允许电池路径接通。

外部电源判定：

- 存在阈值：`BMS_POWER_EXTERNAL_PRESENT_MV = 10000mV`
- 回差：`BMS_POWER_EXTERNAL_HYSTERESIS_MV = 1000mV`

电池充满判定：

- 若状态机进入 `BMS_CHARGE_STATE_DONE`，视为已满。
- 或最低单体电压达到目标单体电压减 `20mV`。

## 8. 充电状态机

文件：`charge_manager.c`

状态：

- `IDLE`
- `PRECHECK`
- `TRICKLE`
- `CC`
- `CV`
- `DONE`
- `FAULT`

命令：

- `0x01`：开始充电
- `0x02`：停止充电
- `0x03`：急停
- `0x04`：清故障
- `0x05`：模式切换

模式：

- `AUTO`
- `TRICKLE`
- `CC`
- `CV`

故障收集来源：

1. AFE 单体/总压/温度/芯片保护。
2. ADC 输入、输出、电流、温度。
3. FAULT_OC 硬件过流。
4. AFE ALERT。
5. 急停命令。
6. FUSE、短路和永久保护状态。

清故障规则：

- 只有当前实时故障全部消失，`CMD_CLEAR_FAULT` 才会清除锁存故障。
- 若 AFE/ADC 仍失败，清故障会返回 `BMS_CMD_ERROR_FAULT_ACTIVE`。

## 9. 功率控制

文件：`power_control.c`

功能：

- 初始化 HRTIMER PWM。
- 根据目标电压/电流和 ADC 反馈执行慢速 PI。
- 在 Buck、Boost、过渡区之间映射四开关占空比。
- 任何停止或故障路径统一调用 `power_control_stop()`。

功率级映射：

- Vin 明显高于目标电压：Buck 区，输入侧 PWM，输出侧高边近似常通。
- Vin 明显低于目标电压：Boost 区，输入侧高边近似常通，输出侧低边 PWM。
- Vin 接近目标电压：过渡区，两侧共同动作。

当前闭环是 20 ms 监督型闭环，不是高带宽数字电源环。它适合低压低流验证和竞赛功能演示。

## 10. SOC 与均衡

### 10.1 SOC：`soc_estimator.c`

SOC 不是直接测量量，因此当前实现是估算：

1. 首次拿到有效 BM2016 电芯电压时，用平均单体电压初始化。
2. 充电电流为正且周期非 0 时，用真实输出电流做库仑积分。
3. 无充电电流时，缓慢向真实电压估算值靠拢。
4. 若 AFE 无有效电压，SOC 保持 0，等待后续真实电芯电压恢复。

线性近似：

- `3.0V -> 0%`
- `4.2V -> 100%`

### 10.2 均衡：`balance_manager.c`

只在 `CV` 或 `DONE` 阶段允许均衡。

条件：

- 单体压差 `cellDeltaMv >= balanceDeltaMv`
- 高于最低单体 `balanceDeltaMv` 的电芯进入均衡位图。

均衡位图最终写入 BM2016。

## 11. 通信协议

文件：

- 固件：`Service/bms_protocol.c/.h`
- 固件通信服务：`Service/bms_comm_service.c`
- 上位机：`BMSMonitorSrc/protocol.cpp/.h`

帧格式：

```text
AA 55 | version | type | sequence | payload_length | payload | crc16_le
```

CRC：

- CRC16/MODBUS
- 覆盖 `version/type/sequence/length/payload`
- 不覆盖 `AA55`

### 11.1 实时状态帧：TYPE 0x01

负载长度：`28` 字节。

| 字段 | 长度 | 单位 |
|---|---:|---|
| timestampMs | 4 | ms |
| packVoltageMv | 2 | mV |
| chargeCurrentMa | 2 | mA |
| inputVoltageMv | 2 | mV |
| dutyX100 | 2 | % * 100 |
| socX10 | 2 | % * 10 |
| chargeState | 1 | 枚举 |
| faultBitmap | 4 | bit map |
| temperaturesX10[4] | 8 | 0.1 摄氏度 |
| chargeMode | 1 | 枚举 |

`chargeMode` 追加在负载尾部，固件和上位机已同步解析。

### 11.2 单体电压帧：TYPE 0x02

负载长度：`26` 字节。

| 字段 | 长度 | 单位 |
|---|---:|---|
| cellMv[9] | 18 | mV |
| cellMaxMv | 2 | mV |
| cellMinMv | 2 | mV |
| cellDeltaMv | 2 | mV |
| balanceBitmap | 2 | bit map |

### 11.3 参数帧：TYPE 0x11 / 0x12

负载长度：`14` 字节。

| 字段 | 长度 | 单位 |
|---|---:|---|
| targetVoltageMv | 2 | mV |
| targetCurrentMa | 2 | mA |
| cutoffCurrentMa | 2 | mA |
| cellOvpMv | 2 | mV |
| cellUvpMv | 2 | mV |
| tempOtpX10 | 2 | 0.1 摄氏度 |
| balanceDeltaMv | 2 | mV |

### 11.4 ACK：TYPE 0x7F

负载长度：`4` 字节。

| 字段 | 长度 |
|---|---:|
| ackType | 1 |
| ackSequence | 1 |
| result | 1 |
| errorCode | 1 |

## 12. 上位机对应修改

上位机当前与固件协议保持同步：

1. `RealTimeStatusData` 增加 `chargeMode`。
2. `parseRealTimeStatus()` 按 28 字节解析。
3. 界面充电状态显示为 `chargeState / chargeMode`。
4. CSV 增加 `charge_mode` 列。
5. 移除本地演示遥测按钮和定时器。
6. 串口连接后等待并显示下位机真实帧。

## 13. 主要源码文件职责

| 文件 | 职责 |
|---|---|
| `User/main.c` | 主入口、时钟、板级初始化、启动 RTOS |
| `User/gd32g5x3_it.c` | 外部故障中断、USART2 TX 中断 |
| `App/app_tasks.c` | FreeRTOS 任务编排、首帧真实采样、周期控制 |
| `Config/bms_board_config.h` | 主控板/功率板硬件映射和标定参数 |
| `Common/bms_types.h` | 全局枚举、状态结构、故障位 |
| `Service/bms_state.c` | 控制任务与通信任务之间的状态快照 |
| `Driver/afe_gd30bm2016.c` | BM2016 SPI、保护配置、电芯电压、FET、均衡 |
| `Driver/adc_manager.c` | 功率板 ADC 真实采样和物理量换算 |
| `Service/safety_manager.c` | 最高优先级故障锁存和快速关断 |
| `Module/power_path_manager.c` | Q4/Q5 电池路径策略 |
| `Module/charge_manager.c` | 充电状态机、命令处理、故障汇总 |
| `Driver/power_control.c` | HRTIMER PWM、Buck/Boost 映射、慢速 PI |
| `Module/balance_manager.c` | 被动均衡位图计算 |
| `Module/soc_estimator.c` | 基于真实电压和真实电流的 SOC 估算 |
| `Service/bms_protocol.c` | 二进制协议编解码和 CRC |
| `Service/bms_comm_service.c` | 串口帧收发调度、周期主动上报 |
| `BSP/bms_uart.c` | USART2 非阻塞 RX/TX 中断环形缓冲 |
| `BMSMonitorSrc/protocol.cpp` | 上位机协议解析与构帧 |
| `BMSMonitorSrc/data.cpp` | 上位机真实遥测显示 |
| `BMSMonitorSrc/serial.cpp` | 上位机串口连接与接收 |

## 14. 已删除或移除的内容

正式主工程中已删除：

- `Driver/board_test.c`
- `Driver/board_test.h`
- Keil 工程中的 `board_test.c/.h` 文件引用
- AFE 电芯电压软件造数逻辑
- ADC 输入电压、电流、温度软件造数逻辑
- `afe_gd30bm2016_mock_set_charging()`
- 上位机本地演示遥测按钮和定时器

保留在 `GD32G553_power` 的测试工程仍可作为硬件 bring-up 参考，但不再进入主工程正式构建。

## 15. 当前编译结果

固件：

```text
Program Size: Code=44332 RO-data=1808 RW-data=8 ZI-data=23824
"..\Output\GD32G553_CHG.axf" - 0 Error(s), 0 Warning(s).
```

上位机：

```text
[100%] Built target BMSMonitor
```

## 16. 后续上板检查建议

1. 烧录主工程后，先不开充电，只连接上位机，确认 100 ms 内收到实时状态帧，500 ms 内收到单体电压帧。
2. 若 BM2016 未接或通信异常，上位机应显示 AFE 通信故障，单体电压为 0。
3. 若功率 ADC 未接或转换失败，上位机应显示 ADC 采样异常，Vin/Vout/Iin/Iout 为 0。
4. 接入真实电池后，确认 9 串电芯电压与万用表/AFE 工具读数一致。
5. 接入电源板后，确认 Vin/Vout/Iin/Iout 与测试电源板已验证数据一致。
6. 拉低/触发 AFE ALERT，确认 Q4/Q5 立即关闭，故障锁存。
7. 触发 FAULT_OC，确认 HRTIMER 输出和 BM2016 FET 路径均关闭。
8. 故障信号仍存在时发送清故障，应返回故障仍存在；故障解除后才允许清除。
