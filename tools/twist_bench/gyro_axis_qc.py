from __future__ import annotations

"""Optional raw-gyro principal-direction QC for one sensor and movement block.

This answers whether angular velocity is concentrated around one 3-D direction.
The returned vector is in the sensor frame; it is not an anatomical axis and its
sign is arbitrary. Final bend/twist estimation still uses calibrated orientation.
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import twist_bench_v0 as v0


def principal_direction(gyro: np.ndarray) -> dict[str, object]:
    gyro = np.asarray(gyro, dtype=float)
    if gyro.ndim != 2 or gyro.shape[1] != 3 or len(gyro) < 5:
        raise ValueError("gyro must be an N x 3 array with at least five rows")
    centered = gyro - np.mean(gyro, axis=0)
    covariance = centered.T @ centered / max(1, len(centered) - 1)
    values, vectors = np.linalg.eigh(covariance)
    order = np.argsort(values)[::-1]
    values = np.maximum(values[order], 0.0)
    axis = vectors[:, order[0]]
    if axis[int(np.argmax(np.abs(axis)))] < 0:
        axis = -axis
    total = float(np.sum(values))
    fractions = values / total if total > 0 else np.zeros(3)
    rms = np.sqrt(np.mean(centered * centered, axis=0))
    return {
        "principal_axis_sensor_xyz": axis.tolist(),
        "eigenvalues": values.tolist(),
        "energy_fractions": fractions.tolist(),
        "dominance_fraction_lambda1_over_sum": float(fractions[0]),
        "planarity_fraction_lambda1_plus_lambda2_over_sum": float(fractions[:2].sum()),
        "coordinate_axis_rms_dps": {"x": float(rms[0]), "y": float(rms[1]), "z": float(rms[2])},
        "interpretation": "sensor-frame QC only; rotate through sensor-to-segment calibration before anatomical use",
    }


def load_gyro(path: Path, imu: str, start: float, end: float) -> np.ndarray:
    text = path.read_text(encoding="utf-8", errors="replace")
    records = v0.parse_serial_text(text) or v0.parse_long_table_rows(v0.read_dict_rows(text))
    rows = [
        (record.gx_dps, record.gy_dps, record.gz_dps)
        for record in records
        if record.imu.upper() == imu.upper() and start <= record.t_s <= end
    ]
    if len(rows) < 5:
        raise SystemExit(f"Fewer than five rows for {imu} in {start:g}-{end:g}s")
    return np.asarray(rows, dtype=float)


def self_test() -> None:
    rng = np.random.default_rng(7)
    axis = np.array([1.0, 2.0, 0.5]); axis /= np.linalg.norm(axis)
    amplitude = np.sin(np.linspace(0, 12 * np.pi, 1000)) * 20
    gyro = amplitude[:, None] * axis + rng.normal(0, 0.1, (len(amplitude), 3))
    result = principal_direction(gyro)
    estimated = np.asarray(result["principal_axis_sensor_xyz"])
    assert abs(float(np.dot(axis, estimated))) > 0.999
    assert result["dominance_fraction_lambda1_over_sum"] > 0.99
    print("gyro_axis_qc self-test: PASS")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path)
    parser.add_argument("--imu", default="IMU2")
    parser.add_argument("--start", type=float)
    parser.add_argument("--end", type=float)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.self_test:
        self_test()
        return 0
    if args.input is None or args.start is None or args.end is None:
        raise SystemExit("--input, --start and --end are required (or use --self-test).")
    result = principal_direction(load_gyro(args.input, args.imu, args.start, args.end))
    result.update({"input": str(args.input), "imu": args.imu, "window_s": [args.start, args.end]})
    rendered = json.dumps(result, indent=2, ensure_ascii=False)
    print(rendered)
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(rendered + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
