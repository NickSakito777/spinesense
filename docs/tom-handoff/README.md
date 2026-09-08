# SpineSense repaired-garment test and data-collection kit

This acquisition package is for the Southampton team to test the repaired five-IMU
garment and collect IMU-only movement data without live support from UCL. Start
here, then follow the linked pages in order. Do not add participant data to a
Git repository.

On Windows, double-click `README.html` for the easiest browser view of this
guide. The `.md` files contain the same instructions.

## Download from GitHub

1. On the repository homepage, select **Code > Download ZIP**.
2. In File Explorer, right-click the downloaded ZIP and choose **Extract All**.
3. Open the extracted repository, then `docs`, then `tom-handoff`. Copy the entire `tom-handoff` folder to `C:\SpineSense` and rename that copy to `tom`.
4. Confirm this file is now `C:\SpineSense\tom\README.md`. Open `README.html` in your browser for the offline tutorial, or read this Markdown version on GitHub.
5. Follow BUILD.md first. The included `wheelhouse` supplies the Python acquisition dependency; Python, Arduino IDE and STM32CubeProgrammer still require their documented installers.

Use the firmware inside this folder with its matching wiring table. The repository's older top-level firmware and README describe a different development snapshot. This kit contains no participant recordings or trained classification model. Never commit the generated `data` directory, photos or session records to GitHub.

## What a successful handoff contains

The target is:

- a hardware acceptance record showing that all five sensors stayed connected;
- one short rehearsal session, checked before any participant session;
- one formal round from each of 4–6 coded participants;
- at least one additional round after that participant completely removes and
  puts the garment on again;
- photos, placement mapping, tap calibration, notes, raw timestamped serial data,
  cue timestamps and a QC report for every donning/round.

There is no motion-capture step. The package does not train or score a model.
UCL performs all later processing.

## Read and run in this order

1. [BUILD.md](BUILD.md) — install the Windows software, identify the board and
   COM port, check the wiring, and flash the two supplied firmware sketches.
2. [docs/hardware-acceptance.md](docs/hardware-acceptance.md) — test the repair
   and approve or reject the garment for human data collection.
3. [docs/operator-guide.md](docs/operator-guide.md) — create the Python
   environment and learn the five commands used during collection.
4. [protocol/session-plan.md](protocol/session-plan.md) — run the rehearsal and
   formal participant rounds.
5. [docs/troubleshooting.md](docs/troubleshooting.md) — use this decision tree
   whenever a step fails.
6. [protocol/data-return.md](protocol/data-return.md) — check, copy and package
   the data for offline return.

Print [docs/print-checklist.md](docs/print-checklist.md) for the collection desk.
Copy the fillable text in [docs/forms.md](docs/forms.md) when a form needs to be
restored.

## Package map

| Path | Purpose |
|---|---|
| `firmware/imu_i3c_connectivity_5/` | Five-sensor connection test at 115200 baud |
| `firmware/imu_i3c_xyz_sflp/` | Five-sensor stream at 921600 baud with automatic bus recovery |
| `tools/session.py` | Creates sessions, finds ports, captures data, presents cues and checks results |
| `tools/review_taps.py` | Makes an offline tap-trace page for manual mapping review |
| `BUILD.md` | Windows installation, wiring and firmware flashing |
| `docs/hardware-acceptance.md` | Repaired-garment acceptance procedure |
| `docs/operator-guide.md` | Beginner command guide and expected files |
| `docs/troubleshooting.md` | Stop/retry/continue decision tree |
| `docs/print-checklist.md` | Short printable desk checklist |
| `docs/forms.md` | Replacement placement and notes templates |
| `protocol/session-plan.md` | Rehearsal and formal collection procedure |
| `protocol/data-return.md` | QC, backup, privacy and return instructions |

## Fixed hardware facts for this package

The controller is a **NUCLEO-U385RG-Q**. The supplied firmware uses the board's
ST-LINK virtual COM port (USART1), not native USB CDC. The serial speeds are
115200 for the connectivity sketch and 921600 for the streaming sketch.

The Nucleo-side wiring used by both supplied sketches is:

| Signal | Nucleo Arduino header |
|---|---|
| SCL | D15 / PB13 |
| SDA | D14 / PB14 |
| VDD and VDDIO | 3V3 |
| GND | GND |
| CS for all sensors | 3V3 (high for I3C mode) |
| IMU0 TA0 | D2 / PC8 |
| IMU1 TA0 | D4 / PB5 |
| IMU2 TA0 | **D6 / PB10** |
| IMU3 TA0 | D8 / PC7 |
| IMU4 TA0 | D9 / PC6 |

**Do not infer the garment connector's 2×5 hole numbering from this table.**
The repaired garment's connector-to-signal map must come from Southampton's
confirmed repair/pinout record. Verify that map before applying power. D6 is
intentional; the earlier D7 route was suspected damaged.

## Safety and data boundaries

- Use only a garment that has passed the hardware acceptance procedure.
- Ask each participant to move only within a range that feels comfortable and
  controlled. They may make a smaller movement or stop at any time. Stop the
  round if they report discomfort, pain, dizziness or unsteadiness. This guide
  does not define clinical range of motion or participant eligibility.
- Use participant codes `SOT01`–`SOT06`, never names or identifying details, in
  filenames, forms and notes.
- Frame garment-position photos to exclude the face and any identifying items.
- Keep data in the approved project storage locations used by the Southampton
  and UCL teams. Do not use public repositories or public links.

## Evidence status

The firmware contains field patches written for STM32 Arduino core 2.12.0:
slower address assignment, retries and automatic recovery after transient bus
loss. Software-only tests can verify the host tools and file rules. Only the
Southampton team can mark the repaired garment, Windows PC, COM-port capture and
real participant runs as passed; the supplied forms keep those results explicit.
