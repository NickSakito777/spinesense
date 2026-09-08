# Troubleshooting decision tree

Preserve every partial session. Never edit a raw `.log`, timing CSV, capture JSON,
cue CSV or cue JSON. Record what happened in `notes.txt`.

## The board does not appear

1. Does `session.py ports` show a new STLink virtual COM port after plug-in?
   - **No:** close Arduino IDE, try the Nucleo ST-LINK connector, a known USB
     data cable and a different PC USB socket. Check Device Manager. Reinstall
     the ST-LINK driver from STM32CubeProgrammer if Windows shows an unknown
     device.
   - **Yes:** write down the new COM number and use it exactly, for example
     `COM7`.
2. If upload also fails, confirm the board is NUCLEO-U385RG-Q, the upload method
   is STM32CubeProgrammer (SWD), and the ST-LINK section is powered.

## `PermissionError`, `Access is denied`, or port already in use

Close Arduino Serial Monitor, Arduino Serial Plotter and every other capture
window. Only one process can own the virtual COM port. Wait five seconds and
retry in a **newly created session directory** if a previous capture already
wrote files.

## Upload succeeds but serial output is blank or unreadable

- Connectivity firmware uses 115200 baud; streaming firmware uses 921600 baud.
- Press reset once after opening Serial Monitor.
- Confirm the selected port disappears when the board is unplugged.
- Confirm `U(S)ART support = Enabled (generic Serial)` and `USB support = None`.
- The stream uses the ST-LINK VCP, not native USB CDC.

## Connectivity is below five sensors or changes when the cable moves

Stop participant work. Power down. Check the adapter is fully seated and
strain-relieved, then repeat the unpowered inspection and full connectivity
test. Do not change garment connector hole assignments or firmware pins by
guessing. A repeatable movement-dependent failure returns to repair.

## Streaming prints `READ_FAIL`, `RECOVER`, or a missing-sensor warning

- A startup warning that an IMU is not streaming: stop, reflash/run connectivity
  firmware, and require all five sensors.
- Any observed interval without all five sensors during cues: stop and review
  the round even if the automated result is only a warning. The checker blocks
  missing full-interval coverage, per-sensor receipt gaps over its conservative
  gate, invalid sensor samples and clock resets; other incomplete-frame counts
  remain warnings for human judgment. Preserve the files, fix the hardware
  problem and repeat in a new session if appropriate.
- A `RECOVER begin/done` pair means self-recovery was attempted. Record it. Do
  not treat recovery as proof the missing interval is scientifically usable.

## Cue program refuses to start

- Confirm Window A says `RECORDING` for the **same session path**.
- Confirm Window A is still receiving data and its `imu_timing.csv` is updating.
- Run both programs on the same physical computer.
- If capture stopped or the wrong directory was used, preserve the folder and
  create a new session. Do not combine capture from one folder with cues from
  another.

## Cues were interrupted, read incorrectly, or the participant made an error

Press Ctrl+C in the capture window after making the situation safe. Keep all
files. Record the last reliable bout and the mistake in notes. Create a new
session if repeating the round; never rerun cues into the same folder. If only
one bout was performed incorrectly and the round is not repeated, identify the
bout for UCL rather than altering timestamps.

## Capture stops, PC sleeps, USB disconnects, or MCU time restarts

The round is blocked. Keep the PC connected to mains power and disable automatic
sleep before the next attempt. Fix the cable/port issue, pass connectivity again,
then create a new session. A backwards MCU clock creates a new timestamp segment
and is deliberately blocked by QC.

## `check` says `BLOCKED`

Read the listed critical errors and the generated `qc_*.json`. Do not delete the
folder or call it passed. If the problem is repeatable, correct the cause and run
the entire donning/round again in a new directory. Return both attempts.

## `check` says `REVIEW_REQUIRED`

This is expected. Review the report, `placement.csv`, photos, tap order, cue
completion and notes. Confirm all five sensors cover the cue interval and that
the session identity is consistent. Write `ACCEPT FOR RETURN` or `REJECT/REPEAT`
with your initials and reason in `notes.txt`. Automated checks cannot verify
movement quality, actual body position or scientific validity.
