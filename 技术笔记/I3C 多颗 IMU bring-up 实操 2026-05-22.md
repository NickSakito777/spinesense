# Multi-IMU I3C Bring-up Procedure (2026-05-22; N=3–5 Hardware Testing)

> **Historical note — 22 May 2026.** This translation preserves the original claims, calculations, code and test scope. It is not a new validation of electrical limits or repaired-garment operation. The historical TA0 mapping below includes **IMU2 on D7/PA8**; do not substitute it for the current kit mapping. Use the [Tom handoff guide](../docs/tom-handoff/README.md) for the current firmware and wiring.

> Builds on the N-device SETDASA pseudocode in [stm32duino I3C feasibility research, 2026-05-22](stm32duino%20I3C%20%E5%8F%AF%E8%A1%8C%E6%80%A7%E8%B0%83%E7%A0%94%202026-05-22.md), covering the wiring, GPIO assignments, pull-up decisions and incremental verification needed to expand from one IMU to N=3–5. **Hardware test on 22 May: N=5 all PASS** (see §3.1).

## One-sentence summary

Physical wiring for 3–5 IMUs consists of four shared buses (3V3 / GND / SCL=PB13 / SDA=PB14) plus N independent TA0 control lines (for N=5, D2/D4/D7/D8/D9 = PC8/PB5/PA8/PC7/PC6). The SETDASA loop processes each device by “drive the addressed device's TA0 high → use static address 0x6B → assign a dynamic address”. **Hardware test on 22 May: N=5 connectivity ALL_PASS; the internal pull-up alone was sufficient, with no external pull-ups fitted.**

---

## 1. Confirmed key facts

| Item | Value | Source |
|---|---|---|
| **TA0=0 (GND)** | Static address 0x6A | `ism6hg256x_reg.h`, `ISM6HG256X_I2C_ADD_L = 0xD5` >>1 |
| **TA0=1 (VDDIO)** | Static address 0x6B | `ISM6HG256X_I2C_ADD_H = 0xD7` >>1 |
| Default (pin left disconnected) | **Do not leave it floating** — TA0 is a high-Z input and will float; it must be fixed to a GPIO | General datasheet guidance |
| ST example combining multi-target SETDASA with GPIO TA0 switching | **None found** — checked ism6hg256x_STdC / lsm6dsv16x_STdC / the 31 I3C examples for STM32CubeU3 NUCLEO-U385RG-Q | Direct GitHub directory inspection |
| Template source | N=5 SETDASA loop in [stm32duino I3C feasibility research, 2026-05-22](stm32duino%20I3C%20%E5%8F%AF%E8%A1%8C%E6%80%A7%E8%B0%83%E7%A0%94%202026-05-22.md) | Project-written code, reduced to N=3 |

**Choice of TA0 polarity**: this note uses “drive TA0 high → address 0x6B”, consistent with the single-device 0x6B approach in 2026-05-22 work log (private source; not included), Part 3.3. Tom's 28 April pseudocode (2026-04-28 work log (private source; not included), Part 1.3) uses the opposite approach, “drive TA0 low → address 0x6A”. The two are functionally equivalent. This note does not adopt that earlier polarity, to keep single- and multi-device logic consistent.

---

## 2. Nucleo-U385RG-Q GPIO assignments (three TA0 control lines)

Source: the `digitalPin[]` array in `Arduino_Core_STM32/variants/STM32U3xx/U375R(E-G)TxQ_U385RGTxQ/variant_NUCLEO_U385RG_Q.cpp`.

**Already in use; avoid these pins**: D14=PB14 (I3C2_SDA) / D15=PB13 (I3C2_SCL) / D13=PA5 (LED) / D28=PC13 (USER_BTN) / D37=PA9 (VCP_TX) / D38=PA10 (VCP_RX).

**Five recommended GPIOs** (used in the 22 May hardware test; N=5 all PASS):

| Purpose | Arduino | STM32 pin | Physical location | Notes |
|---|---|---|---|---|
| IMU0 TA0 | **D2** | PC8 | CN5 pin 3 | Arduino UNO area; independent, with no multiplexing conflict |
| IMU1 TA0 | **D4** | PB5 | CN5 pin 5 | As above; adjacent and easy to wire |
| IMU2 TA0 | **D7** | PA8 | CN5 pin 8 | As above |
| IMU3 TA0 | **D8** | PC7 | CN5 pin 9 | As above |
| IMU4 TA0 | **D9** | PC6 | CN5 pin 10 | **The `PWM` silkscreen on D9 does not prevent ordinary GPIO output** (confirmed experimentally) |

Alternatives, if using the CN10 morpho header: D33=PC2 / D34=PC3 / D36=PD2.

---

## 3. Physical wiring table

| Nucleo CN/Pin | Signal | IMU0 (STEVAL JP/Pin) | IMU1 | IMU2 | IMU3 | IMU4 |
|---|---|---|---|---|---|---|
| 3V3 (CN6-4) | VDD + VDDIO | JP1 pin1+pin2 | Shared | Shared | Shared | Shared |
| GND (CN6-6/7) | GND | JP2 pin13 | Shared | Shared | Shared | Shared |
| **D15 = PB13** | I3C2 SCL | JP2 pin20 | Parallel | Parallel | Parallel | Parallel |
| **D14 = PB14** | I3C2 SDA | JP2 pin21 | Parallel | Parallel | Parallel | Parallel |
| **D2 = PC8** | TA0_IMU0 | JP2 pin22 | — | — | — | — |
| **D4 = PB5** | TA0_IMU1 | — | JP2 pin22 | — | — | — |
| **D7 = PA8** | TA0_IMU2 | — | — | JP2 pin22 | — | — |
| **D8 = PC7** | TA0_IMU3 | — | — | — | JP2 pin22 | — |
| **D9 = PC6** | TA0_IMU4 | — | — | — | — | JP2 pin22 |

**Wiring recommendations**:

- A **breadboard** is preferable to directly joining jumper wires. Three IMUs share four buses (3V3/GND/SCL/SDA) and have three independent TA0 lines; breadboard buses are much more stable than a star of jumper connections.
- Keep total flying-wire length ≤ 30 cm (three IMUs, each with approximately 10 cm of jumper wire), leaving margin relative to the marginal N=5/60 cm case.
- **No additional decoupling capacitors are required** — each STEVAL-MKI248AA board already has 100nF + 2.2μF + 100nF (confirmed in the 18 May note, §1.2).

---

## 4. Pull-up resistor assessment (N=3 + 30 cm)

Using the formula from the 18 May note, `t_rise = 0.8473 × R × C`:

| Item | Estimate |
|---|---|
| Pin capacitance of three IMUs | 3 × (5–10 pF) × 2 lines ≈ 30–60 pF |
| 30 cm flying wire × 1–2 pF/cm | 30–60 pF |
| MCU pin + package | ~5 pF |
| **Total bus C** | **65–125 pF** (midpoint ~95 pF) |

| RPU | Bus C | t_rise | Compared with I3C OD ~150 ns |
|---|---|---|---|
| 2 kΩ (U385 internal) | 95 pF | **161 ns** | Slightly over, by ~7% |
| 2 kΩ | 125 pF (worst case) | 212 ns | Over by 40% |
| 1.5 kΩ (external ⫽ internal ≈ 857 Ω) | 95 pF | **97 ns** | Pass |

**Conclusion**: the estimate puts N=3/30 cm approximately 7% beyond the I3C OD specification. **The 22 May N=5 connectivity hardware test returned ALL_PASS without external pull-ups**, indicating that the estimate was conservative (actual rise time < 150 ns, or IMU tolerance higher than the specification). Confirm experimentally once the physical 60 cm garment topology is available.

**Practical recommendations**:

- **Use only the internal pull-up for the first bring-up round**. After it works, measure the SCL/SDA rising edges with an oscilloscope / Saleae Logic (≥100 MHz probe).
  - < 150 ns → OK; no external pull-up needed.
  - 150–300 ns → add an external 1.5 kΩ resistor.
  - > 300 ns → inspect the wiring.
- **Keep a pair of 1.5 kΩ 0805 through-hole resistors and two female jumper leads available**, so they can quickly be connected in parallel from SCL to 3V3 and SDA to 3V3. **Do not solder them permanently**; keep connections removable during bring-up.

---

## 5. N=3 SETDASA timing pseudocode

```c
// === N=3 SETDASA loop (reduced from the N=5 expansion template in the 22 May feasibility note) ===
const uint8_t DYN_ADDR[3] = {0x32, 0x33, 0x34};

typedef struct { GPIO_TypeDef *port; uint16_t pin; } gpio_t;
const gpio_t TA0[3] = {
  {GPIOC, GPIO_PIN_8},   // IMU0 — D2 / PC8
  {GPIOB, GPIO_PIN_5},   // IMU1 — D4 / PB5
  {GPIOA, GPIO_PIN_8},   // IMU2 — D7 / PA8
};

// Inside setup():
// 1. Use the entire HAL_I3C initialization template from the 22 May stm32duino I3C feasibility note
//    (including the internal pull-up, I3C2 and GPIO mux).
// 2. Configure all three TA0 lines as GPIO_MODE_OUTPUT_PP / NOPULL, initially PIN_RESET
//    (all at 0x6A; no conflict while they are not being addressed).

static uint8_t  setdasa_data[1];
static uint32_t ctrlBuf[16];
static uint8_t  txBuf[16];

for (int i = 0; i < 3; i++) {
  // 1. Drive only device i's TA0 high: its static address is 0x6B; the others remain at 0x6A.
  for (int j = 0; j < 3; j++)
    HAL_GPIO_WritePin(TA0[j].port, TA0[j].pin,
                      (i == j) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_Delay(1);  // Wait for the IMU's internal address latch (actually < 100 µs; 1 ms is safe).

  // 2. Send SETDASA to static address 0x6B to assign dynamic address DYN_ADDR[i].
  setdasa_data[0] = (DYN_ADDR[i] << 1);
  I3C_CCCTypeDef directCCC[] = {
    { 0x6B, Direct_SETDASA, {setdasa_data, 1}, LL_I3C_DIRECTION_WRITE }
  };
  I3C_XferTypeDef xfer = { {ctrlBuf, 16}, {txBuf, 1}, {NULL, 0} };
  HAL_I3C_AddDescToFrame(&hi3c2, directCCC, NULL, &xfer, 1,
                         I3C_DIRECT_WITHOUT_DEFBYTE_STOP);
  HAL_I3C_Ctrl_TransmitCCC_IT(&hi3c2, &xfer);
  while (HAL_I3C_GetState(&hi3c2) != HAL_I3C_STATE_READY) { }

  // 3. Verify that the device has received its dynamic address and acknowledges it.
  if (HAL_I3C_Ctrl_IsDeviceI3C_Ready(&hi3c2, DYN_ADDR[i], 300, 1000) != HAL_OK) {
    VCP.printf("IMU%d SETDASA failed at static 0x6B\n", i);
    while (1);  // Stop here; do not proceed to the next device and risk an inconsistent bus state.
  }
  VCP.printf("IMU%d -> DYN 0x%02X OK\n", i, DYN_ADDR[i]);

  // 4. After assignment, drive this TA0 low again, matching the others and preventing
  //    an unintended response to 0x6B on the next iteration.
  //    Its dynamic address is now latched, so it no longer listens at the static address.
  HAL_GPIO_WritePin(TA0[i].port, TA0[i].pin, GPIO_PIN_RESET);
}
// All three devices now have addresses 0x32 / 0x33 / 0x34.
// Subsequent HAL_I3C_Ctrl_Transmit_IT / _Receive_IT register accesses use the dynamic addresses directly.
```

**Do not rewrite the HAL_I3C initialization code** — copy the complete template at lines 94–156 of [stm32duino I3C feasibility research, 2026-05-22](stm32duino%20I3C%20%E5%8F%AF%E8%A1%8C%E6%80%A7%E8%B0%83%E7%A0%94%202026-05-22.md) (I3C2 initialization + internal pull-up + GPIO AF6 mux).

---

## 5b. Actual code excerpts (22 May hardware test: ALL_PASS)

The complete 415-line code is in `SpineSense FYP/firmware/imu_i3c_connectivity_5/imu_i3c_connectivity_5.ino`. The following four excerpts show the key differences between the production implementation and the pseudocode in §5.

### IRQ-driven operation and flag synchronization (replacing busy-waiting)

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

`HAL_NVIC_EnableIRQ(I3C2_EV_IRQn / I3C2_ER_IRQn)` must be configured inside `HAL_I3C_MspInit`.

### DISEC + RSTDAA broadcast pair (not RSTDAA alone)

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

DISEC first disables IBI / Hot-Join, then RSTDAA clears dynamic addresses. This prevents IBI configuration from the previous run from interfering with the current SETDASA sequence.

### WHO_AM_I private read: `I3C_PRIVATE_WITH_ARB_RESTART` + `MultipleTransfer_IT`

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

Writing the register address and reading the data use an arbitration restart (repeated start), not a stop followed by a start.

### Main loop (N=5 connectivity scan)

```c
static void runConnectivityTest() {
  setAllTa0Low();
  if (!resetDynamicAddresses()) { VCP.println("RSTDAA: FAIL"); return; }
  uint8_t passCount = 0;
  for (uint8_t i = 0; i < IMU_COUNT; i++) {
    selectOnlyTa0(i);                          // Drive only device i's TA0 high.
    if (!setDynamicAddress(imus[i].dynAddr)) continue;
    setAllTa0Low();                            // Drive all TA0 lines low after assigning this device.
    if (HAL_I3C_Ctrl_IsDeviceI3C_Ready(&hi3c2, imus[i].dynAddr, 300, 1000) != HAL_OK) continue;
    if (!readReg(imus[i].dynAddr, WHO_AM_I_REG, rxBuf, 1)) continue;
    if (rxBuf[0] == 0x73) passCount++;
  }
}
```

Four key production choices:

- **IT mode** with `waitI3CReady` flag synchronization, rather than polling `HAL_I3C_GetState`.
- Broadcast **DISEC before RSTDAA**, together.
- Use `I3C_PRIVATE_WITH_ARB_RESTART` for **private register-addressed reads**.
- Call `setAllTa0Low()` after each address assignment, rather than driving only the current device low, to avoid a residual TA0 state in the next iteration.

---

## 6. Incremental verification: N=1 → N=2 → ... → N=5

| Stage | Wiring | Verification objective | Pass criterion |
|---|---|---|---|
| **N=1a smoke** | One IMU + TA0 soldered directly to 3V3 (not connected to a GPIO) | HAL_I3C initialization + single-device SETDASA, 0x6B → 0x32 | `IsDeviceI3C_Ready(0x32)` returns HAL_OK; reading WHO_AM_I returns the ISM6HG256X ID |
| **N=1b GPIO** | One IMU + TA0 connected to D2/PC8 | Verify GPIO control of TA0: low → responds at 0x6A; high → responds at 0x6B; must not respond at both simultaneously | `IsDeviceI3C_Ready(0x6A)` and `0x6B` are mutually exclusive |
| **N=2** | Two IMUs + TA0 connected to D2/D4 | SETDASA loop above (i=0,1) + parallel WHO_AM_I reads from both devices | Both receive independent dynamic addresses; parallel 100 Hz reads have no packet loss |
| **N=2 oscilloscope** | As above | Measure SCL/SDA rise time | < 150 ns passes; > 150 ns requires external 1.5 kΩ pull-ups |
| **N=3** | Three IMUs + D2/D4/D7 | Complete loop + parallel sampling from three IMUs at 200 Hz | Bus error count stays at 0 for five minutes |
| **N=4** | Add D8/PC7 | Extend the above | All four return WHO_AM_I=0x73 |
| **N=5** | Add D9/PC6 | Extend the above to the final garment configuration | **22 May hardware test: ALL_PASS, dynamic addresses 0x32–0x36** |

**Fallback when a stage fails**:

- If N=2 stalls, do not advance to N=3. Enable VCP output and print the status of each step first.
- NACK during the N=2 OD phase → inspect rise time with an oscilloscope first; a pull-up problem is the most likely cause.
- After N=2 SETDASA, 0x32 works but 0x33 fails → the second TA0 GPIO is probably wired incorrectly, or the IMU has a poor solder joint.

---

## 7. Key URLs

- [ISM6HG256X datasheet](https://www.st.com/resource/en/datasheet/ism6hg256x.pdf) — address-table source.
- [Nucleo-U385RG-Q variant files](https://github.com/stm32duino/Arduino_Core_STM32/tree/main/variants/STM32U3xx/U375R(E-G)TxQ_U385RGTxQ) — GPIO-mapping source.
- [STM32CubeU3 single-device I3C SETDASA example](https://github.com/STMicroelectronics/STM32CubeU3/tree/main/Projects/NUCLEO-U385RG-Q/Examples/I3C/I3C_Sensor_Private_Command_IT) — the only official ST reference identified.
- Multi-target SETDASA + GPIO TA0 example: **none found** after checking ism6hg256x_STdC / lsm6dsv16x_STdC / the 31 I3C examples for STM32CubeU3 NUCLEO-U385RG-Q. A custom implementation is required.

---

## Related notes

- [stm32duino I3C feasibility research, 2026-05-22](stm32duino%20I3C%20%E5%8F%AF%E8%A1%8C%E6%80%A7%E8%B0%83%E7%A0%94%202026-05-22.md) — complete HAL_I3C initialization template; required reading first.
- [Pull-up resistors and the I3C physical layer, 2026-05-18](%E4%B8%8A%E6%8B%89%E7%94%B5%E9%98%BB%E4%B8%8E%20I3C%20%E7%89%A9%E7%90%86%E5%B1%82%E8%B0%83%E7%A0%94%202026-05-18.md) — source of the rise-time formula and bus-capacitance estimate.
- 2026-04-28 work log (private source; not included), Part 2.4 — Tom's corrected addr_assign.c pseudocode; opposite TA0 polarity, not adopted here.
- [STEVAL-MKI248KA commercial IMU specifications](STEVAL-MKI248KA%20%E5%95%86%E4%B8%9A%E7%89%88IMU%E8%A7%84%E6%A0%BC.md) — DIL24 pinout and physical TA0 location.
- 2026-05-22 work log (private source; not included), Part 3.3 — decision to use static 0x6B for the single-device case, retained here.

---

**Research date**: 2026-05-22.

**Research method**: one agent combined existing vault research with WebFetch access to official ST repositories and the Arduino_Core_STM32 variant file.

**Precision note**: the 161 ns rise-time estimate versus the 150 ns specification used a 2 kΩ order-of-magnitude estimate for the STM32U385 internal pull-up resistance, as stated in the original 18 May note; it is not a measured typical value from the datasheet. **The 22 May test with N=5 and approximately 30 cm of flying wire returned connectivity ALL_PASS using only the internal pull-up**, meaning that the actual rise time was < 150 ns or that ISM6HG256X tolerance of I3C OD edges exceeded the specification. Confirm experimentally once the physical 60 cm garment topology is available.
