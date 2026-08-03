from __future__ import annotations

"""Zero-lag, leave-one-bout-out accuracy for already aligned block samples.

Input CSV must contain group columns (default: movement), bout_id, imu_deg and
mocap_deg. Upstream code is responsible for clock alignment, block selection,
neutral/re-tare and extracting the chosen IMU readout. This scorer never searches
lag on a held-out bout.
"""

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

import numpy as np


def fit_line(x: np.ndarray, y: np.ndarray) -> tuple[float, float]:
    if len(x) < 2 or np.std(x) < 1e-9:
        raise ValueError("calibration requires at least two non-constant IMU samples")
    gain, offset = np.polyfit(x, y, 1)
    return float(gain), float(offset)


def rmse(error: np.ndarray) -> float:
    return float(np.sqrt(np.mean(error * error)))


def score_bouts(bouts: dict[str, tuple[np.ndarray, np.ndarray]]) -> tuple[dict[str, object], list[dict[str, object]]]:
    if len(bouts) < 2:
        raise ValueError("leave-one-bout-out scoring requires at least two bouts")
    x = np.concatenate([pair[0] for pair in bouts.values()])
    y = np.concatenate([pair[1] for pair in bouts.values()])
    if np.std(x) < 1e-9 or np.std(y) < 1e-9:
        raise ValueError("IMU and MoCap signals must both vary")

    gain, offset = fit_line(x, y)
    fitted = gain * x + offset
    raw_error = x - y
    fitted_error = fitted - y
    rom = float(np.ptp(y))

    folds: list[dict[str, object]] = []
    fold_gains: list[float] = []
    for held_id, (test_x, test_y) in bouts.items():
        train = [pair for bout_id, pair in bouts.items() if bout_id != held_id]
        train_x = np.concatenate([pair[0] for pair in train])
        train_y = np.concatenate([pair[1] for pair in train])
        fold_gain, fold_offset = fit_line(train_x, train_y)
        fold_gains.append(fold_gain)
        prediction = fold_gain * test_x + fold_offset
        for index, (imu, mocap, pred) in enumerate(zip(test_x, test_y, prediction)):
            folds.append({
                "bout_id": held_id,
                "sample_index": index,
                "imu_deg": float(imu),
                "mocap_deg": float(mocap),
                "oof_prediction_deg": float(pred),
                "oof_error_deg": float(pred - mocap),
                "fold_gain": fold_gain,
                "fold_offset": fold_offset,
            })

    oof_error = np.asarray([row["oof_error_deg"] for row in folds], dtype=float)
    bias = float(np.mean(oof_error))
    sd = float(np.std(oof_error, ddof=1)) if len(oof_error) > 1 else 0.0
    raw_score = 1.0 - rmse(raw_error) / max(rom, 1e-9)
    heldout_score = 1.0 - rmse(oof_error) / max(rom, 1e-9)
    result = {
        "n_bouts": len(bouts),
        "n_samples": len(x),
        "pooled_r": float(np.corrcoef(x, y)[0, 1]),
        "gain": gain,
        "offset_deg": offset,
        "gain_range_loo": [float(np.min(fold_gains)), float(np.max(fold_gains))],
        "rom_deg": rom,
        "raw_rmse_deg": rmse(raw_error),
        "raw_mae_deg": float(np.mean(np.abs(raw_error))),
        "in_sample_calibrated_rmse_deg": rmse(fitted_error),
        "in_sample_calibrated_mae_deg": float(np.mean(np.abs(fitted_error))),
        "heldout_rmse_deg": rmse(oof_error),
        "heldout_mae_deg": float(np.mean(np.abs(oof_error))),
        "raw_accuracy_fraction": raw_score,
        "heldout_accuracy_fraction": heldout_score,
        "accuracy_basis": "native" if raw_score >= 0.6 and 0.7 <= gain <= 1.4 else "gain_corrected_only",
        "oof_bland_altman_bias_deg": bias,
        "oof_bland_altman_loa95_deg": [bias - 1.96 * sd, bias + 1.96 * sd],
        "calibration_scope": "leave-one-bout-out within each supplied group; no held-out lag search",
    }
    return result, folds


def load_groups(path: Path, group_cols: list[str], bout_col: str, imu_col: str, mocap_col: str):
    grouped: dict[tuple[str, ...], dict[str, list[tuple[float, float]]]] = defaultdict(lambda: defaultdict(list))
    with path.open(newline="", encoding="utf-8-sig") as fh:
        reader = csv.DictReader(fh)
        required = {*group_cols, bout_col, imu_col, mocap_col}
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise SystemExit(f"Missing CSV column(s): {', '.join(sorted(missing))}")
        for line_number, row in enumerate(reader, 2):
            try:
                imu = float(row[imu_col])
                mocap = float(row[mocap_col])
            except (TypeError, ValueError) as exc:
                raise SystemExit(f"Invalid numeric value on CSV line {line_number}") from exc
            if not np.isfinite(imu) or not np.isfinite(mocap):
                raise SystemExit(f"Non-finite value on CSV line {line_number}")
            key = tuple(row[column] for column in group_cols)
            grouped[key][row[bout_col]].append((imu, mocap))
    return grouped


def self_test() -> None:
    bouts = {}
    for index in range(3):
        x = np.linspace(-5, 5, 40) + index
        y = 2.0 * x + 1.0
        bouts[f"rep{index + 1}"] = (x, y)
    result, folds = score_bouts(bouts)
    assert result["heldout_rmse_deg"] < 1e-10
    assert abs(result["gain"] - 2.0) < 1e-10
    assert len(folds) == 120
    print("cleaned_accuracy self-test: PASS")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path)
    parser.add_argument("--group-cols", default="movement", help="Comma-separated grouping columns, e.g. subject,movement.")
    parser.add_argument("--bout-col", default="bout_id")
    parser.add_argument("--imu-col", default="imu_deg")
    parser.add_argument("--mocap-col", default="mocap_deg")
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--oof-csv", type=Path, help="Write out-of-fold predictions for Bland-Altman/QC.")
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.self_test:
        self_test()
        return 0
    if args.input is None:
        raise SystemExit("--input is required (or use --self-test).")
    group_cols = [x.strip() for x in args.group_cols.split(",") if x.strip()]
    groups = load_groups(args.input, group_cols, args.bout_col, args.imu_col, args.mocap_col)
    report: dict[str, object] = {}
    oof_rows: list[dict[str, object]] = []
    for key, raw_bouts in groups.items():
        bouts = {
            bout_id: (np.asarray([x for x, _ in rows]), np.asarray([y for _, y in rows]))
            for bout_id, rows in raw_bouts.items()
        }
        label = " / ".join(key)
        try:
            result, folds = score_bouts(bouts)
        except ValueError as exc:
            result, folds = {"error": str(exc), "n_bouts": len(bouts)}, []
        report[label] = result
        for row in folds:
            oof_rows.append({**{column: value for column, value in zip(group_cols, key)}, **row})
        if "error" in result:
            print(f"{label}: ERROR {result['error']}")
        else:
            print(
                f"{label}: n={result['n_bouts']} r={result['pooled_r']:.3f} gain={result['gain']:.3f} "
                f"rawRMSE={result['raw_rmse_deg']:.2f} heldRMSE={result['heldout_rmse_deg']:.2f} deg "
                f"[{result['accuracy_basis']}]"
            )

    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    if args.oof_csv and oof_rows:
        args.oof_csv.parent.mkdir(parents=True, exist_ok=True)
        with args.oof_csv.open("w", newline="", encoding="utf-8") as fh:
            writer = csv.DictWriter(fh, fieldnames=list(oof_rows[0]))
            writer.writeheader()
            writer.writerows(oof_rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
