# IMU Fanout Board — Design Requirements Brief

> **Historical note.** This document preserves the May 2026 fanout-board design
> brief and should not be used as current wiring instructions. In particular,
> the statements below that unused pins including CS may remain unconnected and
> that a selected TA0 is pulled low describe an earlier proposal and conflict
> with the current field firmware/wiring. Use
> [the Tom handoff](../docs/tom-handoff/README.md) for current acquisition and
> wiring instructions.

> This is the requirements description for a small custom PCB in the SpineSense
> FYP project.

---

## 1. What this project is

SpineSense is a wearable spinal-posture monitoring system our group is working
on. Put simply, it is a T-shirt with five IMUs (inertial measurement units) sewn
along the spine on the back. The IMUs detect spinal bending and rotation in real
time for at-home scoliosis screening and monitoring.

**The role of this custom PCB in the system**:

```
[STM32 Nucleo-U385RG-Q controller board]
              │
              │ a ribbon-cable bundle
              ▼
       [the custom board]   ← this is the board to design
              │
       5 separate interfaces
              │
              ▼
       one IMU plugged into each
```

In essence, it is a **signal fanout board**: one bundle of wires comes in from
the controller, and the board fans that bundle out into five branches, one for
each IMU. **The board does not need any active components** (no MCU and no logic
IC); it is a routing-only PCB.

---

## 2. Selected hardware (all datasheets are attached)

| Role | Model | Key documents |
|---|---|---|
| Controller board | STM32 Nucleo-U385RG-Q (board number MB1841E) | UM3062 user manual + STM32U385RG datasheet |
| IMU × 5 | STEVAL-MKI248KA (ST commercial IMU evaluation kit containing the ISM6HG256X) | STEVAL-MKI248KA Data Brief (DB5602) + ISM6HG256X datasheet |
| How the IMU connects to the board | The commercial kit includes a **DIL24 adapter (STEVAL-MKIGIBV5)**, which plugs directly into a DIL24 socket | See the Data Brief, Figure 1 + Figure 2 schematic |

→ **The board therefore needs five 24-pin DIL24 sockets**, allowing the five IMU
adapters to plug in directly.

---

> **⚠️ Note about the sections below**
>
> Sections 3–6 describe our current concept for this board: how signals should
> be routed, why TA0 should be independent, where the pull-up resistors should
> be placed, and how acceptance should be performed. We compiled these points
> from the datasheets we had and our project experience over the preceding few
> months, **but they may not be accurate and may not be the best solution**.
>
> Please first read this document and the attached PDFs. We can then discuss and
> align on how the board should actually be built. **If anything in our concept
> appears unreasonable or insufficiently rigorous, or if there is a better
> approach, please use your engineering judgment rather than accommodating our
> written details.** This document describes “approximately what we want to do
> plus several engineering constraints.”

---

## 3. What the board needs to do

Passively fan out one I3C bus from the controller to five IMUs, so all five IMUs
operate on the same I3C bus.

The controller will send **20 signal wires** into the board. These 20 wires fall
into **two routing categories**:

### Shared signals (5 wires, shorted together on the board and fanned out to all 5 sockets)

| Signal | Description |
|---|---|
| VDD | IMU core supply, 3.3 V |
| VDDIO | IMU I/O supply, 3.3 V (may share the same trace as VDD) |
| GND | Ground |
| SCL | I3C clock |
| SDA | I3C data |

For each of these five signals, the corresponding pin on every DIL24 socket is
**shorted to the same trace**. The five IMUs share the same SCL/SDA pair and the
same power/ground rails.

### Independent signals (15 wires, routed separately for each of the 5 sockets)

| Signal | Quantity | Why it must be independent |
|---|---:|---|
| **SDO/SA0/TA0** (DIL24 pin 22) | 5 | **Determines each IMU's address**—see Section 4; this is critical |
| INT1 (DIL24 pin 14) | 5 | Interrupt signal; each IMU independently tells the controller “I have an event” |
| INT2 (DIL24 pin 15) | 5 | As above, for the second interrupt line |

Each of these 15 signals has an **independent trace from its socket** back to the
controller-side connector. These pins must **never be shorted together between
the five sockets**.

The remaining DIL24 pins (CS, SCX, SDX, OSDO, OCS, and so on) are unused and can
be **left NC (floating)**.

---

## 4. Engineering constraint: **why five independent TA0 wires are needed**

**Problem:** all five ISM6HG256X devices are the same model, and their default
I²C addresses provide only two choices, 0x6A and 0x6B. They cannot be uniquely
identified when placed on the same bus.

**Our solution:** use the I3C **SETDASA command** (CCC 0x87) together with each
IMU's **TA0 pin** (that is, SDO/SA0, DIL24 pin 22). During initialization, the
controller “calls” each device one at a time:

1. The controller pulls IMU #1's TA0 low → IMU #1's default address becomes
   0x6A, while the other four remain at 0x6B.
2. The controller sends SETDASA to 0x6A → only IMU #1 responds and receives an
   independent dynamic address (for example, 0x50).
3. After IMU #1 receives its dynamic address, it no longer responds to SETDASA
   (an I3C protocol rule).
4. The controller repeats the process for IMUs #2–5, assigning 0x51–0x54.
5. The controller then communicates normally with each IMU using its dynamic
   address.

**Implications for the board design:**

- The controller must be able to **control the five IMU TA0 pins independently**.
  Therefore, each TA0 must have its own trace and **must not be shorted together
  on the board**.
- If they are shorted, the five TA0 pins will always have the same level and the
  devices will always have the same address. The entire addressing mechanism
  will fail and the completed board will be unusable.

---

## 5. Other constraints

### 5.1 Pull-up resistors

The I3C physical layer needs **SDA/SCL pull-up resistors** (to VDDIO = 3.3 V).

Please place one **0402 SMD resistor** on each of the SDA and SCL traces. A value
between 1 kΩ and 2.2 kΩ is suggested (this value does not have to be followed
exactly).

> Background: neither the commercial STEVAL-MKI248KA nor the Nucleo board has
> onboard SDA/SCL pull-ups, so these two resistors can only be placed on our
> custom board.

### 5.2 Physical layout / garment integration

The arrangement of the five DIL24 sockets on the board is open (linear row,
compact matrix, or another arrangement). For context, the board will eventually
be **mounted on or near the garment** (with devices placed along the spine), so
smaller is better.

### 5.3 Controller-side connector

The connector between the board and Nucleo is open to choice, but it should be
stable and needs only to carry the 20 signals plus power/ground while allowing
convenient assembly and removal. The garment will be assembled and disassembled
often, so a keyed or locking connector is preferable.

### 5.4 Onboard component list

Only the following are allowed on the complete board:

- **2 × SDA/SCL pull-up 0402 resistors** (Section 5.1)
- **5 × 24-pin DIL24 sockets**
- **1 × controller-side connector**
- routing traces, pads, and silkscreen

**No other components** (no buffer, mux, level shifter, or decorative LED).
Keep the board as passive, inexpensive, and easy to manufacture as possible.

### 5.5 PCB manufacturing process

Any suitable process is acceptable.

---

## 6. Acceptance criteria

The board is considered acceptable when:

1. **Visual inspection:** the five DIL24 sockets, two SMD resistors, and
   controller connector are soldered correctly, with no cold joints.
2. **Multimeter continuity test:** the five shared signals have continuity to
   the same corresponding pin on every socket; the 15 independent signals have
   no continuity between sockets and connect only to their corresponding pins
   on the controller-side connector.
3. **Functional test:** with five IMUs and the Nucleo connected, SETDASA
   initialization gives all five devices independent dynamic addresses, and the
   controller can read six-axis data from each.

The designer/board manufacturer is responsible for Steps 1–2. I will perform
Step 3 myself (it requires the firmware to be ready and does not block board
delivery).

---

## 7. Attachment list

I will send the following PDFs with this document. They contain the detailed
technical information:

| Document | Purpose |
|---|---|
| STEVAL-MKI248KA Data Brief (DB5602) | IMU kit schematic and DIL24 pinout |
| ISM6HG256X datasheet | Pin definitions and electrical characteristics of the IMU IC |
| STM32U385RG datasheet + UM3062 | Controller-board pins and voltage specifications (primarily to confirm 3.3 V I/O compatibility) |
| | |

---

## 8. Contact

Please ask me about anything that is unclear—especially the signal categories
in Section 3 and the reason TA0 is independent in Section 4. These are central
to the board; if they are misunderstood, the completed board will not work.

For other engineering details (board dimensions, routing rules, connector
model, PCB process, and layout aesthetics), I trust your judgment. Please use
the approach you consider appropriate.

---

**Document version:** v1 (2026-05-18)
**Project:** SpineSense FYP (UCL Rehabilitation Engineering & Assistive Technologies)
