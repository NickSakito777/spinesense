from __future__ import annotations

import argparse
import datetime as dt
import sys
import threading
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parent
DEPS_DIR = ROOT.parent / "imu_cube_viewer" / ".deps"
if DEPS_DIR.exists():
    sys.path.insert(0, str(DEPS_DIR))


def main() -> int:
    parser = argparse.ArgumentParser(description="Capture SpineSense two-IMU serial output to a raw log file.")
    parser.add_argument("--serial", default="COM3", help="Serial port. Default: COM3.")
    parser.add_argument("--baud", type=int, default=921600, help="Serial baud rate. Default: 921600 (5-IMU SFLP firmware).")
    parser.add_argument("--seconds", type=float, default=7200.0, help="Safety ceiling (s). Default: 2 hours. Press Ctrl-C in the terminal to stop when the whole session is done.")
    parser.add_argument("--output", type=Path, help="Output log path.")
    parser.add_argument(
        "--cue-twist",
        action="store_true",
        help="Play simple timing cues for a 0 -> +30 -> 0 -> -30 -> 0 twist trial.",
    )
    args = parser.parse_args()

    try:
        import serial
    except ModuleNotFoundError as exc:
        raise SystemExit("pyserial not found. The imu_cube_viewer .deps folder should contain it.") from exc

    out_path = args.output or default_output_path()
    out_path.parent.mkdir(parents=True, exist_ok=True)

    print(f"Opening {args.serial} @ {args.baud}")
    print(f"Writing {out_path}")
    print("Recording. Run the WHOLE session, then press Ctrl-C here to stop (auto-stops at the ceiling).")

    line_count = 0
    sample_count = 0
    deadline = time.monotonic() + args.seconds
    cue_stop = threading.Event()
    if args.cue_twist:
        threading.Thread(target=run_twist_cues, args=(time.monotonic(), cue_stop), daemon=True).start()

    with serial.Serial(args.serial, args.baud, timeout=1) as ser, out_path.open("w", encoding="utf-8", newline="") as log:
        try:
            ser.reset_input_buffer()
            while time.monotonic() < deadline:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").rstrip("\r\n")
                log.write(line + "\n")
                line_count += 1
                if line_count % 200 == 0:
                    log.flush()  # keep data on disk so a hard stop (VSCode stop button) loses ~nothing
                parts = line.strip().split()
                if len(parts) >= 10 and len(parts[1]) >= 4 and parts[1].startswith("IMU"):
                    sample_count += 1
                    if sample_count <= 6 or sample_count % 100 == 0:
                        print(line)
        except KeyboardInterrupt:
            print("\nStopped by user (Ctrl-C).")
        finally:
            cue_stop.set()
            log.flush()

    print(f"Captured {line_count} lines, {sample_count} sample rows.")
    print(out_path)
    return 0


def run_twist_cues(start_time: float, stop: threading.Event) -> None:
    schedule = [
        (0.0, "HOLD 0 deg neutral"),
        (8.0, "MOVE to +30 deg"),
        (13.0, "HOLD +30 deg"),
        (18.0, "RETURN to 0 deg"),
        (23.0, "HOLD 0 deg"),
        (28.0, "MOVE to -30 deg"),
        (33.0, "HOLD -30 deg"),
        (38.0, "RETURN to 0 deg and hold"),
    ]
    for target_s, label in schedule:
        while not stop.is_set() and time.monotonic() - start_time < target_s:
            time.sleep(0.02)
        if stop.is_set():
            return
        print(f"CUE {target_s:04.1f}s: {label}")
        beep(target_s)


def beep(target_s: float) -> None:
    try:
        import winsound

        if target_s == 0.0:
            winsound.Beep(660, 180)
        else:
            winsound.Beep(880, 120)
            time.sleep(0.08)
            winsound.Beep(880, 120)
    except Exception:
        pass


def default_output_path() -> Path:
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return ROOT / "data" / f"twist_trial_{stamp}.log"


if __name__ == "__main__":
    sys.exit(main())
