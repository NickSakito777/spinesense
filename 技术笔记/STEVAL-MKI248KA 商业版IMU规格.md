# STEVAL-MKI248KA — Long-Term Primary IMU Hardware for SpineSense FYP

> **Historical note.** This page preserves the hardware inventory and technical
> understanding recorded during the project. It is not an executable wiring or
> acquisition procedure. Use
> [the Tom handoff](../docs/tom-handoff/README.md) for current acquisition
> instructions and current field-firmware wiring.

> This note records the fixed technical specifications and inventory status of
> the commercial IMU kit used by SpineSense FYP, serving as a context anchor for
> future AI conversations. On 2026-05-17, Fox established this kit as the
> long-term primary platform (see 2026-05-17 work log (private source; not included), Part 1).

## Current inventory

- **6–7 complete STEVAL-MKI248KA kits** (purchased in 2026-05)
- Planned use: 5 sensors for the primary garment path (positioned along the
  spine) + 1–2 spare units
- N = 5 is the number of interfaces on the current custom bus-distribution
  expansion board

## Kit contents (3 items per kit)

| Physical item | Part number | Description |
|---|---|---|
| **Square IMU board** | STEVAL-MKI248AA | Carries the ISM6HG256X chip, provides three connector groups J1/J2/J3, and includes VDD/VDDIO decoupling capacitors |
| **Flat cable** | flat cable | 12-pin (6×2), connects square-board J1 ↔ MKIGIBV5 JP1 |
| **DIL24 adapter** | STEVAL-MKIGIBV5 | Fans the square-board signals out to the standard 24-pin DIL24 interface |

## Three connector specifications (from ST Data Brief DB5602 Rev 1, 2025-09-04)

### Square-board J1 (6×2 header = 12 pins) — basic signal breakout

| Pin | Signal | Description |
|---:|---|---|
| 1 | VDD | Core supply |
| 2 | VDDIO | I/O supply (3.6 V maximum, 4.6 V absolute maximum) |
| 3 | VDDIO | As above |
| 4 | SA0/SDO | I²C/I3C static-address LSB (TA0) = SPI MISO |
| 5 | SDX | Auxiliary I²C/I3C data |
| 6 | SDA | Main I²C/I3C data = SPI MOSI |
| 7 | SCX | Auxiliary I²C/I3C clock |
| 8 | SCL | Main I²C/I3C clock = SPI SCK |
| 9 | OSDO | Auxiliary SPI MISO |
| 10 | CS | SPI chip select |
| 11 | INT1 | Interrupt 1 |
| 12 | GND | Ground |

Bypass connections: R1 (0 Ω) → OCS (auxiliary SPI chip select); R2 (0 Ω) → INT2
(interrupt 2)

### Square-board J2 (17×2 header = 34 pins) — complete X-NUCLEO-IKS02A1 interface

Includes dual BI (bus interface) / BS (bus selector) bus signals: SCL_BI /
SCL_BS / SAO_SDO / SDA_BS / CS / INT2 / INT1 / SDA_BI, and so on.

### Square-board J3 (5×2 header = 10 pins) — spare breakout

Includes SCL_BS / SDA_BS / SDA_BI / SCL_BI / VDDIO / VDD, and so on.

### MKIGIBV5 DIL24 adapter (JP1 + JP2, 24 pins total)

Standard ST MEMS evaluation-board interface, directly corresponding to the
complete ISM6HG256X pinout (VDDIO / VDD / OCS / SCX / SDX / OSDO / SDA / SCL /
CS / SDO-SAO / INT2 / INT1).

## Key constraints (to prevent incorrect future AI inferences)

### 1. The kit itself does not include SDA/SCL pull-up resistors

The schematic shows only three VDD/VDDIO decoupling capacitors on the square
board: C1 (100 nF), C2 (2.2 µF), and C3 (100 nF). There are **no SDA/SCL
pull-up resistors**. When multiple devices share a bus, pull-ups must be provided
externally (at the controller or expansion board). For a 60 cm bus with five
devices, **2.2 kΩ** is suggested (see 2026-04-28 work log (private source; not included), Part 5.3).

**However**, the STM32U385 controller itself includes dedicated internal I3C
pull-ups (the `PWR_I3CPUCR1/2` registers in the PWR block, with HAL APIs
`HAL_PWREx_EnableI3CPullUp(PWR_I3CPU_PB13/PB14)`, estimated at 1.5–2.2 kΩ).
AN5879's officially recommended approach is to use the internal pull-up rather
than an external one. **The custom SpineSense expansion board still carries 0402
pull-up resistors** as the primary path (60 cm + five devices is a marginal I3C
topology, and populated onboard resistors are more stable), while the internal
pull-up is enabled at the same time for redundancy. **See
[Pull-up resistors and the I3C physical layer: investigation (2026-05-18)](%E4%B8%8A%E6%8B%89%E7%94%B5%E9%98%BB%E4%B8%8E%20I3C%20%E7%89%A9%E7%90%86%E5%B1%82%E8%B0%83%E7%A0%94%202026-05-18.md) for details.**

### 2. More than two identical devices on one bus → I3C SETDASA + TA0 switching is mandatory

- The ISM6HG256X provides only **two I²C static addresses, 0x6A and 0x6B** →
  I²C cannot place more than two identical devices on one bus.
- **I3C ENTDAA arbitration fails because all PID OTP values are 0x09** (measured
  in the 4-28 work log, Part 1.3).
- The only feasible method is **I3C SETDASA (CCC 0x87) + GPIO-controlled TA0
  switching** to give each device an independent dynamic address (4-28 work log,
  Part 7).
- This mechanism requires **each IMU's TA0 (Pin 4) to return independently to an
  MCU GPIO**; they cannot be shorted together on the board.

### 3. Logic-level compatibility with different controllers

- **ISM6HG256X VDDIO maximum 3.6 V, absolute maximum 4.6 V**—a direct 5 V
  connection will cause damage.
- All **Arduino UNO R3** GPIO uses 5 V logic → a level shifter is required.
- The **Arduino UNO R4 WiFi / Minima** main GPIO still uses 5 V logic (the RA4M1
  is designed for 5 V operation and compatibility with the R3), but the board
  has a dedicated Qwiic / STEMMA QT connector (4-pin JST-SH: GND/3.3V/SDA/SCL).
  This is a separate 3.3 V I²C bus and can connect to a 3.3 V sensor without a
  level shifter.
- **ESP32 / ESP32-S3 / Nucleo-U385 / Teensy 4.x** are all 3.3 V compatible and
  can connect directly.

### 4. Arduino library support

- Official library: [stm32duino/ISM6HG256X](https://github.com/stm32duino/ISM6HG256X)
  v2.0.0 (2025-10-27)
- Uses the standard Arduino `TwoWire` + `SPIClass` APIs and is **theoretically
  cross-platform** (ESP32 / SAMD / RP2040 / UNO R4, and so on).
- **The official ISM6HG256X Arduino wrapper supports only I²C and SPI, not
  I3C.** With Nucleo + Arduino IDE, I3C therefore requires calling STM32 HAL_I3C
  directly in the sketch, or later manually importing the `libraries/I3C`
  directory from the stm32duino main branch.

## Compatible controller platforms

| Platform | Supported? | I3C? | I²C? | SPI? | Notes |
|---|---|---|---|---|---|
| STM32U385 Nucleo + STM32 HAL | ✅ | ✅ | ✅ | ✅ | The only route capable of I3C (after the Windows flashing path was unlocked on 5-20) |
| STM32U385 Nucleo + stm32duino | ✅ | ✅ (direct HAL_I3C call in sketch; release 2.12.0 did not include I3CBus) | ✅ | ✅ | Arduino IDE handles compilation/flashing; the I3C communication layer initially connects directly through HAL |
| **Arduino UNO R4 WiFi** | ✅ | ❌ | ✅ (direct 3.3 V connection through Qwiic / level shifting required on the main bus) | ⚠️ (level shifter required) | I²C ≤ 2 devices; onboard ESP32-S3 coprocessor can provide Wi-Fi/BLE uplink |
| Arduino UNO R3 | ⚠️ | ❌ | ⚠️ (level shifter required) | ⚠️ (level shifter required) | I²C ≤ 2 devices |
| ESP32 / ESP32-S3 / ESP32-C3 | ✅ | ❌ | ✅ | ✅ | Directly compatible with 3.3 V |
| ESP32-P4 | ✅ (ESP-IDF only) | ✅ | ✅ | ✅ | The only ESP with I3C, but the Arduino framework does not support it |
| Teensy 4.x | ✅ | ❌ | ✅ | ✅ | 3.3 V compatible; multiple high-speed SPI buses |
| Official STEVAL-MKI109D evaluation motherboard | ✅ | ✅ | ✅ | ✅ | PC bridge + MEMS Studio GUI for no-code verification |
| X-NUCLEO-IKS02A1 / STEVAL-STWINBX1 | ✅ | ✅ | ✅ | ✅ | Direct connection through square-board J2 (34-pin) |

## Related documents

- **Official data brief PDF** (Fox local copy):
  `~/Downloads/steval-mki248ka (1).pdf` (ST DB5602 Rev 1, 2025-09-04)
- Online data brief: https://www.st.com/resource/en/data_brief/steval-mki248ka.pdf
- Product page: https://www.st.com/en/evaluation-tools/steval-mki248ka.html
- ISM6HG256X datasheet: https://www.st.com/resource/en/datasheet/ism6hg256x.pdf
- Arduino library source: https://github.com/stm32duino/ISM6HG256X
- Historical decision: 2026-05-17 work log (private source; not included), Part 1 (establishing the
  long-term primary platform) + Part 4 (Arduino multi-board research results) +
  Part 6 (Day 1 flashing blocker)
- Historical background: 2026-04-28 work log (private source; not included) (Tom bare-chip PCB route +
  origin of the SETDASA solution + Table 25 evidence)

## In progress: custom bus-distribution expansion board (from 5-17)

A domestic board manufacturer will produce a custom expansion board to fan out
one MCU bus to five STEVAL-MKI248KA devices.

| Confirmed specification | Value |
|---|---|
| Number of interfaces, N | **5** |
| Physical form | **A** (five 24-pin DIL24 sockets on the board, allowing direct insertion of the MKIGIBV5 adapters) |
| INT1 / INT2 | Independently broken out for each device (redundancy) |
| Number of signals | **20** (5 shared: VDD/VDDIO/GND/SCL/SDA + 5 independent TA0 + 10 independent INT1/INT2) |
| Onboard components | None (routing-only PCB) |
| Protocol layer | I3C SETDASA + GPIO-controlled TA0 switching (derived from Constraint 2) |
| Controller-side interface | To be decided (20-pin IDC / 2×10-pin / FFC ribbon cable) |

| Specification still to be decided (discussion in progress on the evening of 5-17) |
|---|
| Layout of the five DIL24 sockets (linear row / compact matrix / custom irregular arrangement) |
| Location of SDA/SCL pull-up resistors (external at MCU / unpopulated SMD pads on board / populated resistors on board, violating the “no components” principle) |
| Form of the MCU-side connector (20-pin IDC ribbon / two grouped 10-pin connectors / FFC ribbon cable) |
