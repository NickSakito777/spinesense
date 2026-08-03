# STEVAL-MKI248KA — SpineSense FYP 长期主线 IMU 硬件

> 本笔记是 SpineSense FYP 商业版 IMU 套件的固定技术规格 + 持有状态，作为未来 AI 对话的上下文锚点。Fox 2026-05-17 决定将此套件确立为长期主线（参 [[2026-05-17 - 工作日志]] Part 1）。

## 当前持有

- **6-7 块** STEVAL-MKI248KA 完整套件（2026-05 购入）
- 规划用途：5 颗用于 garment 主线（沿脊柱布置）+ 1-2 颗 spare 备份
- N=5 = 当前定制总线分发拓展板的接口数

## 套件构成（每套 3 件）

| 物理实体 | 编号 | 描述 |
|---|---|---|
| **IMU 方板** | STEVAL-MKI248AA | 焊有 ISM6HG256X 芯片，提供 J1/J2/J3 三组接口，板上已有 VDD/VDDIO 去耦电容 |
| **扁平排线** | flat cable | 12-pin (6×2)，连接方板 J1 ↔ MKIGIBV5 JP1 |
| **DIL24 adapter** | STEVAL-MKIGIBV5 | 把方板信号扇出成标准 DIL24 socket（24-pin） |

## 三种连接器规格（来自 ST data brief DB5602 Rev 1, 2025-09-04）

### 方板 J1（Header 6×2 = 12-pin）—— 基本信号引出

| Pin | 信号 | 说明 |
|---|---|---|
| 1 | VDD | 核心电源 |
| 2 | VDDIO | IO 电源（最高 3.6V，绝对最大 4.6V） |
| 3 | VDDIO | 同上 |
| 4 | SA0/SDO | I²C/I3C 静态地址 LSB（TA0）= SPI MISO |
| 5 | SDX | 辅助 I²C/I3C 数据 |
| 6 | SDA | 主 I²C/I3C 数据 = SPI MOSI |
| 7 | SCX | 辅助 I²C/I3C 时钟 |
| 8 | SCL | 主 I²C/I3C 时钟 = SPI SCK |
| 9 | OSDO | 辅助 SPI MISO |
| 10 | CS | SPI 片选 |
| 11 | INT1 | 中断 1 |
| 12 | GND | 地 |

旁路：R1（0Ω）→ OCS（辅助 SPI 片选）；R2（0Ω）→ INT2（中断 2）

### 方板 J2（Header 17×2 = 34-pin）—— X-NUCLEO-IKS02A1 完整接口

包含 BI（bus interface）/ BS（bus selector）双路总线信号：SCL_BI / SCL_BS / SAO_SDO / SDA_BS / CS / INT2 / INT1 / SDA_BI 等。

### 方板 J3（Header 5×2 = 10-pin）—— 备用引出

包含 SCL_BS / SDA_BS / SDA_BI / SCL_BI / VDDIO / VDD 等。

### MKIGIBV5 DIL24 adapter（JP1 + JP2 共 24-pin）

标准 ST MEMS 评估底板接口，直接对应 ISM6HG256X 完整 pinout（VDDIO / VDD / OCS / SCX / SDX / OSDO / SDA / SCL / CS / SDO-SAO / INT2 / INT1）。

## 关键约束（防止未来 AI 误判）

### 1. 套件本身不带 SDA/SCL 上拉电阻
schematic 显示方板上只有 C1（100nF）/ C2（2.2µF）/ C3（100nF）三个 VDD/VDDIO 去耦电容，**没有 SDA/SCL pull-up resistor**。多颗共总线时必须在外部（主控端或拓展板）提供上拉，60cm 总线 + 5 颗时建议 **2.2 kΩ**（参 [[2026-04-28 - 工作日志]] Part 5.3）。

**但**——主控 STM32U385 自身内置 I3C dedicated pull-up（PWR 块的 `PWR_I3CPUCR1/2` 寄存器，HAL API `HAL_PWREx_EnableI3CPullUp(PWR_I3CPU_PB13/PB14)`，估值 1.5-2.2 kΩ），AN5879 官方推荐做法是用 internal pull-up 不用外部。**SpineSense 定制拓展板上仍贴 0402 上拉电阻**作为主路径（60cm + 5 颗是 I3C marginal 拓扑，板上贴更稳），internal pull-up 同时启用作冗余。**详见 [[上拉电阻与 I3C 物理层调研 2026-05-18]]**。

### 2. 同型号 N>2 颗共总线 → 强制走 I3C SETDASA + TA0 切换
- ISM6HG256X 的 **I²C 静态地址只有 0x6A / 0x6B 两个** → I²C 无法挂 > 2 颗同型号
- **I3C ENTDAA 因 PID OTP 都 0x09 仲裁失败**（4-28 工作日志 Part 1.3 实测）
- 唯一可行：**I3C SETDASA(CCC 0x87) + GPIO 切 TA0** 给每颗独立动态地址（4-28 工作日志 Part 7）
- 这条机制要求 **每颗 IMU 的 TA0 (Pin 4) 必须独立引回 MCU GPIO**，不能在板上短接

### 3. 不同主控的电平兼容性
- **ISM6HG256X VDDIO 最高 3.6V，绝对最大 4.6V**——5V 直接接会损坏
- **Arduino UNO R3** 全部 GPIO 5V 逻辑 → 必须电平转换器
- **Arduino UNO R4 WiFi / Minima** 主 GPIO 仍是 5V 逻辑（RA4M1 设计为 5V 操作，与 R3 兼容），但**板上专门有 Qwiic / STEMMA QT 连接器（JST-SH 4-pin: GND/3.3V/SDA/SCL），是独立的 3.3V I²C 总线，接 3.3V 传感器无需电平转换器**
- **ESP32 / ESP32-S3 / Nucleo-U385 / Teensy 4.x** 全部 3.3V 兼容，直接接

### 4. Arduino 库支持
- 官方库：[stm32duino/ISM6HG256X](https://github.com/stm32duino/ISM6HG256X) v2.0.0（2025-10-27）
- 用标准 Arduino `TwoWire` + `SPIClass` API，**理论可跨平台**（ESP32 / SAMD / RP2040 / UNO R4 等）
- **官方 ISM6HG256X Arduino wrapper 只支持 I²C 和 SPI，不支持 I3C**；在 Nucleo + Arduino IDE 下走 I3C 时，做法是在 sketch 里直接调 STM32 HAL_I3C，或后续手动引入 stm32duino main 的 `libraries/I3C`。

## 兼容主控平台

| 平台 | 是否支持 | 走 I3C? | 走 I²C? | 走 SPI? | 备注 |
|---|---|---|---|---|---|
| STM32U385 Nucleo + STM32 HAL | ✅ | ✅ | ✅ | ✅ | 唯一能走 I3C 的路径（5-20 Windows 解锁烧录通道后） |
| STM32U385 Nucleo + stm32duino | ✅ | ✅（HAL_I3C sketch 直调；release 2.12.0 未带 I3CBus） | ✅ | ✅ | Arduino IDE 负责编译/烧录，I3C 通信层先用 HAL 直连 |
| **Arduino UNO R4 WiFi** | ✅ | ❌ | ✅（Qwiic 3.3V 直连 / 主总线需电平转换） | ⚠️（需电平转换器） | I²C ≤ 2 颗；板载 ESP32-S3 协处理器可做 WiFi/BLE 上行 |
| Arduino UNO R3 | ⚠️ | ❌ | ⚠️（需电平转换器） | ⚠️（需电平转换器） | I²C ≤ 2 颗 |
| ESP32 / ESP32-S3 / ESP32-C3 | ✅ | ❌ | ✅ | ✅ | 3.3V 直接兼容 |
| ESP32-P4 | ✅（仅 ESP-IDF） | ✅ | ✅ | ✅ | 唯一带 I3C 的 ESP，但 Arduino 框架不支持 |
| Teensy 4.x | ✅ | ❌ | ✅ | ✅ | 3.3V 兼容；多 SPI 高速 |
| 官方 STEVAL-MKI109D 评估底板 | ✅ | ✅ | ✅ | ✅ | PC bridge + MEMS Studio GUI 无代码验证 |
| X-NUCLEO-IKS02A1 / STEVAL-STWINBX1 | ✅ | ✅ | ✅ | ✅ | 通过方板 J2 (34-pin) 直接对接 |

## 关联文档

- **官方 data brief PDF**（Fox 本地）：`~/Downloads/steval-mki248ka (1).pdf` (ST DB5602 Rev 1 2025-09-04)
- 在线 data brief：https://www.st.com/resource/en/data_brief/steval-mki248ka.pdf
- 产品页：https://www.st.com/en/evaluation-tools/steval-mki248ka.html
- ISM6HG256X datasheet：https://www.st.com/resource/en/datasheet/ism6hg256x.pdf
- Arduino 库源码：https://github.com/stm32duino/ISM6HG256X
- 历史决策：[[2026-05-17 - 工作日志]] Part 1（确立长期主线）+ Part 4（Arduino 多板调研结果）+ Part 6（Day 1 烧录卡点）
- 历史背景：[[2026-04-28 - 工作日志]]（Tom 裸芯片 PCB 路线 + SETDASA 方案诞生 + Table 25 凭据）

## 正在进行：定制总线分发拓展板（5-17 起）

向国内板厂定制一块拓展板，用来把 1 路 MCU 总线扇出到 5 颗 STEVAL-MKI248KA。

| 已定 spec | 值 |
|---|---|
| 接口数 N | **5** |
| 物理形态 | **A**（板上 5 个 24-pin DIL24 socket，让 MKIGIBV5 adapter 直接插入） |
| INT1 / INT2 | 每颗独立引出（冗余） |
| 信号根数 | **20**（5 共享：VDD/VDDIO/GND/SCL/SDA + 5 独立 TA0 + 10 独立 INT1/INT2） |
| 板上元件 | 无（纯走线 PCB） |
| 协议层 | I3C SETDASA + GPIO 切 TA0（约束 2 推导） |
| 主控接口端 | 待定（20-pin IDC / 2×10-pin / FFC 软排线） |

| 待定 spec（5-17 晚间讨论进行中） |
|---|
| 5 个 DIL24 socket 板上布局（一字线性 / 紧凑矩阵 / 自定不规则） |
| SDA/SCL 上拉电阻位置（MCU 端外置 / 板上保留 SMD 空焊位 / 板上贴电阻违反"无器件"原则） |
| MCU 端连接器形式（20-pin IDC 排线 / 2 根 10-pin 分组 / FFC 软排线） |
