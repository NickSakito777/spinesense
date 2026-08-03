# I3C 多颗 IMU bring-up 实操（2026-05-22，N=3-5 全程实测）

> 承接 [[stm32duino I3C 可行性调研 2026-05-22]] 的 N 颗 SETDASA 伪代码，把单颗扩到 N=3-5 的具体接线 + GPIO 分配 + 上拉电阻判断 + 渐进验证步骤。**5-22 实测 N=5 全 PASS**（见 §3.1）。

## 一句话总结

3-5 颗 IMU 物理接线 = 4 条共享母线（3V3 / GND / SCL=PB13 / SDA=PB14）+ N 条独立 TA0 控制线（N=5 时 D2/D4/D7/D8/D9 = PC8/PB5/PA8/PC7/PC6）。SETDASA 循环按"被寻址那颗 TA0 拉高 → 静态 0x6B → 赋动态地址"逐颗执行。**5-22 实测 N=5 connectivity ALL_PASS，internal pull-up 单独足够，未加外部上拉**。

---

## 1. 关键事实确认

| 项 | 值 | 来源 |
|---|---|---|
| **TA0=0 (GND)** | 静态地址 0x6A | `ism6hg256x_reg.h` `ISM6HG256X_I2C_ADD_L = 0xD5` >>1 |
| **TA0=1 (VDDIO)** | 静态地址 0x6B | `ISM6HG256X_I2C_ADD_H = 0xD7` >>1 |
| 默认（pin 悬空） | **不要悬空** — TA0 是 high-Z 输入会浮动，必须固定到 GPIO | datasheet 通则 |
| ST 多 target SETDASA + GPIO TA0 切换 example | **不存在** — 已核对 ism6hg256x_STdC / lsm6dsv16x_STdC / STM32CubeU3 NUCLEO-U385RG-Q I3C 31 个 example | GitHub 直接列目录 |
| 模板来源 | [[stm32duino I3C 可行性调研 2026-05-22]] N=5 SETDASA 循环 | 自写，缩到 N=3 |

**TA0 高低逻辑选择**：本笔记走"TA0 拉高 → 0x6B 寻址"路径（与 [[2026-05-22 - 工作日志]] Part 3.3 单颗 0x6B 一致）。Tom 4-28 伪代码（[[2026-04-28 - 工作日志]] Part 1.3）用的是反向"TA0 拉低 → 0x6A 寻址"，功能等价，本笔记不沿用，避免单颗/多颗逻辑不一致。

---

## 2. Nucleo-U385RG-Q GPIO 分配（3 个 TA0 控制线）

来源：`Arduino_Core_STM32/variants/STM32U3xx/U375R(E-G)TxQ_U385RGTxQ/variant_NUCLEO_U385RG_Q.cpp` 的 `digitalPin[]` 数组。

**已占用（避开）**：D14=PB14 (I3C2_SDA) / D15=PB13 (I3C2_SCL) / D13=PA5 (LED) / D28=PC13 (USER_BTN) / D37=PA9 (VCP_TX) / D38=PA10 (VCP_RX)

**推荐 5 个 GPIO**（5-22 实测使用，N=5 全 PASS）：

| 用途 | Arduino | STM32 pin | 物理位置 | 备注 |
|---|---|---|---|---|
| IMU0 TA0 | **D2** | PC8 | CN5 pin 3 | Arduino UNO 区，单独无复用冲突 |
| IMU1 TA0 | **D4** | PB5 | CN5 pin 5 | 同上，相邻好接线 |
| IMU2 TA0 | **D7** | PA8 | CN5 pin 8 | 同上 |
| IMU3 TA0 | **D8** | PC7 | CN5 pin 9 | 同上 |
| IMU4 TA0 | **D9** | PC6 | CN5 pin 10 | **D9 丝印 `PWM` 不影响普通 GPIO 输出**（实测确认） |

备选（如需走 CN10 morpho 头排针）：D33=PC2 / D34=PC3 / D36=PD2。

---

## 3. 物理接线表

| Nucleo CN/Pin | 信号 | IMU0 (STEVAL JP/Pin) | IMU1 | IMU2 | IMU3 | IMU4 |
|---|---|---|---|---|---|---|
| 3V3 (CN6-4) | VDD + VDDIO | JP1 pin1+pin2 | 共享 | 共享 | 共享 | 共享 |
| GND (CN6-6/7) | GND | JP2 pin13 | 共享 | 共享 | 共享 | 共享 |
| **D15 = PB13** | I3C2 SCL | JP2 pin20 | 并联 | 并联 | 并联 | 并联 |
| **D14 = PB14** | I3C2 SDA | JP2 pin21 | 并联 | 并联 | 并联 | 并联 |
| **D2 = PC8** | TA0_IMU0 | JP2 pin22 | — | — | — | — |
| **D4 = PB5** | TA0_IMU1 | — | JP2 pin22 | — | — | — |
| **D7 = PA8** | TA0_IMU2 | — | — | JP2 pin22 | — | — |
| **D8 = PC7** | TA0_IMU3 | — | — | — | JP2 pin22 | — |
| **D9 = PC6** | TA0_IMU4 | — | — | — | — | JP2 pin22 |

**接线建议**：
- **面包板** 优于 杜邦线直接对接 —— 3 颗 IMU 共享 4 母线（3V3/GND/SCL/SDA）+ 3 独立 TA0，面包板做"母线"比杜邦星形稳得多
- 飞线总长 ≤ 30 cm（3 颗 + 各 ~10 cm 杜邦），与 N=5/60 cm 的 marginal 区拉开余量
- **不需要额外去耦电容** — STEVAL-MKI248AA 每板已自带 100nF + 2.2μF + 100nF（5-18 笔记 §1.2 确认）

---

## 4. 上拉电阻判断（N=3 + 30 cm）

按 5-18 笔记公式 `t_rise = 0.8473 × R × C`：

| 项 | 估算 |
|---|---|
| 3 颗 IMU pin 电容 | 3 × (5-10 pF) × 2 线 ≈ 30-60 pF |
| 飞线 30 cm × 1-2 pF/cm | 30-60 pF |
| MCU pin + package | ~5 pF |
| **总 bus C** | **65-125 pF**（中位 ~95 pF） |

| RPU | bus C | t_rise | vs I3C OD ~150 ns |
|---|---|---|---|
| 2 kΩ（U385 internal） | 95 pF | **161 ns** | 略超 ~7% |
| 2 kΩ | 125 pF（最坏） | 212 ns | 超 40% |
| 1.5 kΩ（外部 ⫽ internal ≈ 857 Ω） | 95 pF | **97 ns** | 通过 |

**结论**：估算显示 N=3/30 cm 略超 I3C OD spec ~7%。**5-22 实测 N=5 connectivity ALL_PASS，未加外部上拉**，证明估算偏保守（实际 rise time < 150 ns 或 IMU 容忍度高于 spec）。garment 60 cm 拓扑实物到手后再实测确认。

**实操建议**：
- **第一轮 bring-up 只用 internal pull-up**，跑通后用示波器 / Saleae Logic（≥100 MHz 探头）实测 SCL/SDA rise edge
  - < 150 ns → OK，不需要外部
  - 150-300 ns → 加外部 1.5 kΩ
  - > 300 ns → 检查接线
- **备一对 1.5 kΩ 0805 直插电阻 + 2 根杜邦母线**，需要时一秒并联到 SCL-3V3 / SDA-3V3 —— **不要焊死**，bring-up 阶段保持可拆

---

## 5. N=3 SETDASA 时序伪代码

```c
// === N=3 SETDASA 循环（基于 5-22 可行性调研笔记 §扩展到 5 颗 模板缩到 N=3） ===
const uint8_t DYN_ADDR[3] = {0x32, 0x33, 0x34};

typedef struct { GPIO_TypeDef *port; uint16_t pin; } gpio_t;
const gpio_t TA0[3] = {
  {GPIOC, GPIO_PIN_8},   // IMU0 — D2 / PC8
  {GPIOB, GPIO_PIN_5},   // IMU1 — D4 / PB5
  {GPIOA, GPIO_PIN_8},   // IMU2 — D7 / PA8
};

// setup() 内：
// 1. 调 [[stm32duino I3C 可行性调研 2026-05-22]] §HAL_I3C 完整初始化代码模板 整段（含 internal pull-up + I3C2 + GPIO mux）
// 2. 把 3 条 TA0 配 GPIO_MODE_OUTPUT_PP / NOPULL，初始全部 PIN_RESET（→ 全部 0x6A，未被寻址不冲突）

static uint8_t  setdasa_data[1];
static uint32_t ctrlBuf[16];
static uint8_t  txBuf[16];

for (int i = 0; i < 3; i++) {
  // 1. 只把第 i 颗的 TA0 拉高 → 该颗静态地址 = 0x6B；其余 = 0x6A
  for (int j = 0; j < 3; j++)
    HAL_GPIO_WritePin(TA0[j].port, TA0[j].pin,
                      (i == j) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_Delay(1);  // 等 IMU 内部地址 latch（实际 < 100 µs，1 ms 安全）

  // 2. 对静态 0x6B 发 SETDASA → 赋动态地址 DYN_ADDR[i]
  setdasa_data[0] = (DYN_ADDR[i] << 1);
  I3C_CCCTypeDef directCCC[] = {
    { 0x6B, Direct_SETDASA, {setdasa_data, 1}, LL_I3C_DIRECTION_WRITE }
  };
  I3C_XferTypeDef xfer = { {ctrlBuf, 16}, {txBuf, 1}, {NULL, 0} };
  HAL_I3C_AddDescToFrame(&hi3c2, directCCC, NULL, &xfer, 1,
                         I3C_DIRECT_WITHOUT_DEFBYTE_STOP);
  HAL_I3C_Ctrl_TransmitCCC_IT(&hi3c2, &xfer);
  while (HAL_I3C_GetState(&hi3c2) != HAL_I3C_STATE_READY) { }

  // 3. 验证：该颗已被赋动态地址，能 ACK
  if (HAL_I3C_Ctrl_IsDeviceI3C_Ready(&hi3c2, DYN_ADDR[i], 300, 1000) != HAL_OK) {
    VCP.printf("IMU%d SETDASA failed at static 0x6B\n", i);
    while (1);  // 停住，不继续下一颗（避免 bus 状态混乱）
  }
  VCP.printf("IMU%d -> DYN 0x%02X OK\n", i, DYN_ADDR[i]);

  // 4. 该颗赋完地址 → TA0 拉回低（与其他保持一致，避免下一轮误响应 0x6B）
  //    它已 latch 动态地址，不再监听静态地址
  HAL_GPIO_WritePin(TA0[i].port, TA0[i].pin, GPIO_PIN_RESET);
}
// 3 颗都拿到 0x32 / 0x33 / 0x34
// 后续 HAL_I3C_Ctrl_Transmit_IT / _Receive_IT 直接用动态地址读寄存器
```

**HAL_I3C init 代码不重写** — 直接照搬 [[stm32duino I3C 可行性调研 2026-05-22]] 第 94-156 行的完整模板（I3C2 init + internal pull-up + GPIO AF6 mux）。

---

## 5b. Actual code 节选（5-22 实测 ALL_PASS）

完整代码 415 行在 `SpineSense FYP/firmware/imu_i3c_connectivity_5/imu_i3c_connectivity_5.ino`。下面 4 段是 production 实现与 §5 伪代码的关键差异。

### IRQ 驱动 + flag 同步（替代 busy wait）

```c
static volatile bool i3cDone = false;
static volatile bool i3cError = false;

extern "C" void I3C2_EV_IRQHandler(void) { HAL_I3C_EV_IRQHandler(&hi3c2); }
extern "C" void I3C2_ER_IRQHandler(void) { HAL_I3C_ER_IRQHandler(&hi3c2); }
extern "C" void HAL_I3C_CtrlTxCpltCallback(I3C_HandleTypeDef *hi3c)          { if (hi3c->Instance==I3C2) i3cDone=true; }
extern "C" void HAL_I3C_CtrlRxCpltCallback(I3C_HandleTypeDef *hi3c)          { if (hi3c->Instance==I3C2) i3cDone=true; }
extern "C" void HAL_I3C_CtrlMultipleXferCpltCallback(I3C_HandleTypeDef *hi3c) { if (hi3c->Instance==I3C2) i3cDone=true; }
extern "C" void HAL_I3C_ErrorCallback(I3C_HandleTypeDef *hi3c)               { if (hi3c->Instance==I3C2) { i3cError=true; i3cDone=true; } }

static bool waitI3CReady(uint32_t timeoutMs) {
  const uint32_t start = millis();
  while (!i3cDone && !i3cError && (millis() - start < timeoutMs)) yield();
  return i3cDone && !i3cError && (HAL_I3C_GetState(&hi3c2) == HAL_I3C_STATE_READY);
}
```

`HAL_I3C_MspInit` 里 `HAL_NVIC_EnableIRQ(I3C2_EV_IRQn / I3C2_ER_IRQn)` 必须配。

### DISEC + RSTDAA broadcast pair（不是单 RSTDAA）

```c
static bool resetDynamicAddresses() {
  uint8_t disecPayload = 0x08;  // bit 3 = disable Hot-Join
  I3C_CCCTypeDef broadcastCcc[] = {
    {0, 0x01 /* DISEC  */, {&disecPayload, 1}, LL_I3C_DIRECTION_WRITE},
    {0, 0x06 /* RSTDAA */, {NULL, 0},          LL_I3C_DIRECTION_WRITE},
  };
  I3C_XferTypeDef xfer = {};
  xfer.CtrlBuf.pBuffer = ctrlBuf; xfer.CtrlBuf.Size = 16;
  xfer.TxBuf.pBuffer  = txBuf;    xfer.TxBuf.Size  = 1;
  HAL_I3C_AddDescToFrame(&hi3c2, broadcastCcc, NULL, &xfer, 2,
                         I3C_BROADCAST_WITHOUT_DEFBYTE_RESTART);
  i3cDone = false; i3cError = false;
  HAL_I3C_Ctrl_TransmitCCC_IT(&hi3c2, &xfer);
  return waitI3CReady(1000);
}
```

DISEC 先禁掉 IBI / Hot-Join，再 RSTDAA 清动态地址。避免上次跑过的 IBI 配置干扰本次 SETDASA。

### WHO_AM_I private read：`I3C_PRIVATE_WITH_ARB_RESTART` + `MultipleTransfer_IT`

```c
static bool readReg(uint8_t dynAddr, uint8_t reg, uint8_t *data, uint16_t len) {
  I3C_PrivateTypeDef desc[2] = {
    {dynAddr, {&reg, 1}, {NULL, 0},   HAL_I3C_DIRECTION_WRITE},
    {dynAddr, {NULL, 0}, {data, len}, HAL_I3C_DIRECTION_READ},
  };
  I3C_XferTypeDef xfer = {};
  xfer.CtrlBuf.pBuffer = ctrlBuf; xfer.CtrlBuf.Size = 2;
  xfer.TxBuf.pBuffer  = txBuf;    xfer.TxBuf.Size  = 1;
  xfer.RxBuf.pBuffer  = data;     xfer.RxBuf.Size  = len;
  HAL_I3C_AddDescToFrame(&hi3c2, NULL, desc, &xfer, 2,
                         I3C_PRIVATE_WITH_ARB_RESTART);
  i3cDone = false; i3cError = false;
  HAL_I3C_Ctrl_MultipleTransfer_IT(&hi3c2, &xfer);
  return waitI3CReady(1000);
}
```

Write 寄存器地址 + Read 数据用 arbitration restart（repeated start），不是 stop 然后 start。

### 主循环（N=5 connectivity scan）

```c
static void runConnectivityTest() {
  setAllTa0Low();
  if (!resetDynamicAddresses()) { VCP.println("RSTDAA: FAIL"); return; }
  uint8_t passCount = 0;
  for (uint8_t i = 0; i < IMU_COUNT; i++) {
    selectOnlyTa0(i);                          // 只把 i 颗 TA0 拉高
    if (!setDynamicAddress(imus[i].dynAddr)) continue;
    setAllTa0Low();                            // 该颗赋完地址后全部 TA0 拉低
    if (HAL_I3C_Ctrl_IsDeviceI3C_Ready(&hi3c2, imus[i].dynAddr, 300, 1000) != HAL_OK) continue;
    if (!readReg(imus[i].dynAddr, WHO_AM_I_REG, rxBuf, 1)) continue;
    if (rxBuf[0] == 0x73) passCount++;
  }
}
```

四个 production 关键选项：
- **IT 模式** + `waitI3CReady` flag 同步，不是 `HAL_I3C_GetState` 轮询
- **DISEC 在 RSTDAA 前**一起广播
- **private read** 用 `I3C_PRIVATE_WITH_ARB_RESTART` 实现 register-addressed read
- 每颗赋完地址后 `setAllTa0Low()`（不只把当前颗拉低，避免下轮 TA0 残留）

---

## 6. N=1 → N=2 → ... → N=5 渐进验证

| 阶段 | 接线 | 验证目标 | 通过判据 |
|---|---|---|---|
| **N=1a smoke** | 1 颗 IMU + TA0 直接焊到 3V3（不接 GPIO） | HAL_I3C init + SETDASA 单颗 0x6B → 0x32 | `IsDeviceI3C_Ready(0x32)` 返回 HAL_OK，读 WHO_AM_I 拿到 ISM6HG256X ID |
| **N=1b GPIO** | 1 颗 IMU + TA0 接 D2/PC8 | 验证 GPIO 控制 TA0 起效（拉低 → 0x6A 响应；拉高 → 0x6B 响应；不能同时响应） | `IsDeviceI3C_Ready(0x6A)` 和 `0x6B` 互斥 |
| **N=2** | 2 颗 IMU + TA0 接 D2/D4 | 上述 SETDASA 循环 (i=0,1) + 两颗 WHO_AM_I 并行读 | 2 颗都拿到独立动态地址；并行读 100 Hz 不丢包 |
| **N=2 示波器** | 同上 | 实测 rise time @ SCL/SDA | < 150 ns 通过；> 150 ns 加 1.5 kΩ 外部 |
| **N=3** | 3 颗 + D2/D4/D7 | 全循环 + 3 颗并行 sample @ 200 Hz | bus error 计数 = 0 持续 5 分钟 |
| **N=4** | + D8/PC7 | 同上扩展 | 4 颗 WHO_AM_I=0x73 |
| **N=5** | + D9/PC6 | 同上扩展，最终 garment 配置 | **5-22 实测 ALL_PASS，dyn addr 0x32-0x36** |

**每阶段失败的 fallback**：
- N=2 卡住别冲 N=3，先打开 VCP 打每步状态
- N=2 OD 段 NACK → 优先示波器看 rise time，多半是上拉问题
- N=2 SETDASA 后 0x32 OK 但 0x33 fail → 大概率第二颗 TA0 GPIO 接线错或 IMU 焊接虚

---

## 7. 关键 URL

- ISM6HG256X datasheet（地址表来源）：https://www.st.com/resource/en/datasheet/ism6hg256x.pdf
- Nucleo-U385RG-Q variant 文件（GPIO 映射来源）：https://github.com/stm32duino/Arduino_Core_STM32/tree/main/variants/STM32U3xx/U375R(E-G)TxQ_U385RGTxQ
- STM32CubeU3 I3C 单颗 SETDASA example（唯一 ST 官方参考）：https://github.com/STMicroelectronics/STM32CubeU3/tree/main/Projects/NUCLEO-U385RG-Q/Examples/I3C/I3C_Sensor_Private_Command_IT
- multi-target SETDASA + GPIO TA0 example：**不存在**（已核对 ism6hg256x_STdC / lsm6dsv16x_STdC / STM32CubeU3 NUCLEO-U385RG-Q I3C 31 个 example），需自写

---

## 关联

- [[stm32duino I3C 可行性调研 2026-05-22]] — HAL_I3C init 完整代码模板（必须先读）
- [[上拉电阻与 I3C 物理层调研 2026-05-18]] — rise time 公式 + bus capacitance 估算来源
- [[2026-04-28 - 工作日志]] Part 2.4 — Tom addr_assign.c 修复版伪代码（反向 TA0 逻辑，本笔记不沿用）
- [[STEVAL-MKI248KA 商业版IMU规格]] — DIL24 pinout + TA0 物理位置
- [[2026-05-22 - 工作日志]] Part 3.3 — 单颗静态 0x6B 路径决定（本笔记沿用）

---

**调研日期**：2026-05-22
**调研方法**：1 个 agent 串联 vault 内已有调研 + WebFetch ST 官方仓库 + Arduino_Core_STM32 variant 文件
**精度提示**：rise time 估算 161 ns vs spec 150 ns 用了 STM32U385 internal pull-up 阻值 2 kΩ 的"量级估算"（5-18 笔记原文），不是 datasheet 实测典型值。**5-22 实测 N=5 + 飞线 ~30 cm 配置下 connectivity ALL_PASS，internal pull-up 单独足够**，意味着实际 rise time < 150 ns 或 ISM6HG256X 对 I3C OD 边沿容忍度大于 spec。garment 60 cm 拓扑实物到手后再实测确认。
