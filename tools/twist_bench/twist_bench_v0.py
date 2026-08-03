from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path


DEFAULT_AXIS = (0.0, 0.0, 1.0)
SERIAL_COLUMNS = (
    "t_ms",
    "imu",
    "position",
    "addr",
    "ax_mg",
    "ay_mg",
    "az_mg",
    "gx_dps",
    "gy_dps",
    "gz_dps",
)


@dataclass(frozen=True)
class ImuSample:
    t_s: float
    imu: str
    ax_mg: float
    ay_mg: float
    az_mg: float
    gx_dps: float
    gy_dps: float
    gz_dps: float
    qw: float | None = None
    qx: float | None = None
    qy: float | None = None
    qz: float | None = None

    @property
    def sflp_quat(self) -> tuple[float, float, float, float] | None:
        if self.qw is None or self.qx is None or self.qy is None or self.qz is None:
            return None
        return (self.qw, self.qx, self.qy, self.qz)


@dataclass(frozen=True)
class PairSample:
    t_s: float
    parent_gyr_dps: tuple[float, float, float]
    child_gyr_dps: tuple[float, float, float]


@dataclass(frozen=True)
class TwistSample:
    t_s: float
    parent_gyr_dps: tuple[float, float, float]
    child_gyr_dps: tuple[float, float, float]
    parent_gyr_cal_dps: tuple[float, float, float]
    child_gyr_cal_dps: tuple[float, float, float]
    rel_axis_dps: float
    twist_deg: float


def main() -> int:
    parser = argparse.ArgumentParser(
        description="SpineSense Part 2 v0: two-IMU relative gyro twist integration."
    )
    parser.add_argument("--input", type=Path, help="Serial log, long CSV/TSV, or wide parent-child CSV.")
    parser.add_argument("--demo", action="store_true", help="Run a synthetic 0 -> 30 -> 0 -> -30 -> 0 trial.")
    parser.add_argument("--parent", default="IMU1", help="Parent/reference IMU id for long logs. Default: IMU1.")
    parser.add_argument("--child", default="IMU2", help="Child/moving IMU id for long logs. Default: IMU2.")
    parser.add_argument(
        "--axis",
        default="0,0,1",
        help="Twist axis in the shared sensor/block frame, comma separated. Default: 0,0,1.",
    )
    parser.add_argument(
        "--bias-seconds",
        type=float,
        default=5.0,
        help="Initial still window used for gyro bias estimation. Default: 5 s.",
    )
    parser.add_argument(
        "--tare-seconds",
        type=float,
        default=5.0,
        help="Initial window whose mean twist is subtracted after integration. Default: 5 s.",
    )
    parser.add_argument(
        "--max-pair-dt-ms",
        type=float,
        default=25.0,
        help="Max parent-child timestamp gap when exact long-log pairing is unavailable. Default: 25 ms.",
    )
    parser.add_argument("--drift-start-s", type=float, help="Start of drift fit window in seconds.")
    parser.add_argument("--drift-end-s", type=float, help="End of drift fit window in seconds.")
    parser.add_argument(
        "--return-window-seconds",
        type=float,
        default=2.0,
        help="Final window used for return-to-zero estimate. Default: 2 s.",
    )
    parser.add_argument("--out-csv", type=Path, help="Output twist CSV path.")
    parser.add_argument("--plot", type=Path, help="Output SVG plot path.")
    parser.add_argument("--summary", type=Path, help="Output JSON summary path.")
    args = parser.parse_args()

    if not args.demo and args.input is None:
        parser.error("Provide --input or use --demo.")

    axis = parse_axis(args.axis)

    if args.demo:
        pairs = make_demo_pairs(axis)
        stem = "demo"
    else:
        pairs = load_pairs(args.input, args.parent, args.child, args.max_pair_dt_ms / 1000.0)
        stem = args.input.stem

    if len(pairs) < 2:
        raise SystemExit("Need at least two paired parent/child samples.")

    samples, detail = integrate_twist(
        pairs=pairs,
        axis=axis,
        bias_seconds=args.bias_seconds,
        tare_seconds=args.tare_seconds,
    )
    summary = summarize(
        samples=samples,
        detail=detail,
        parent=args.parent,
        child=args.child,
        axis=axis,
        drift_start_s=args.drift_start_s,
        drift_end_s=args.drift_end_s,
        return_window_seconds=args.return_window_seconds,
    )

    out_dir = Path(__file__).resolve().parent
    plot_dir = out_dir / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)

    out_csv = args.out_csv or (plot_dir / f"{stem}_twist_v0.csv")
    out_plot = args.plot or (plot_dir / f"{stem}_twist_v0.svg")
    out_summary = args.summary or (plot_dir / f"{stem}_summary.json")

    write_twist_csv(samples, out_csv)
    write_svg_plot(samples, out_plot, title=f"{stem} relative axial twist v0")
    write_json(summary, out_summary)
    print_summary(summary, out_csv, out_plot, out_summary)
    return 0


def parse_axis(text: str) -> tuple[float, float, float]:
    parts = [part.strip() for part in text.split(",")]
    if len(parts) != 3:
        raise SystemExit("--axis must have three comma-separated numbers, e.g. 0,0,1")
    try:
        axis = tuple(float(part) for part in parts)
    except ValueError as exc:
        raise SystemExit(f"Invalid --axis: {text}") from exc
    norm = vector_norm(axis)
    if norm <= 1e-12:
        raise SystemExit("--axis vector must be non-zero.")
    return tuple(value / norm for value in axis)


def load_pairs(path: Path, parent: str, child: str, max_pair_dt_s: float) -> list[PairSample]:
    text = path.read_text(encoding="utf-8", errors="ignore")
    table_rows = read_dict_rows(text)
    if table_rows:
        lower_keys = set(table_rows[0].keys())
        if "imu" in lower_keys and has_any_gyro_column(lower_keys):
            records = parse_long_table_rows(table_rows)
            pairs = pair_long_records(records, parent, child, max_pair_dt_s)
            if pairs:
                return pairs
        pairs = parse_wide_rows(table_rows, parent, child)
        if pairs:
            return pairs

    records = parse_serial_text(text)
    pairs = pair_long_records(records, parent, child, max_pair_dt_s)
    if pairs:
        return pairs

    raise SystemExit(
        "Could not parse input. Expected the current serial table, a long CSV with imu/gx_dps, "
        "or a wide CSV with parent/child gyro columns."
    )


def read_dict_rows(text: str) -> list[dict[str, str]]:
    lines = [line for line in text.splitlines() if line.strip()]
    if not lines:
        return []

    header_index = find_header_line(lines)
    if header_index is None:
        return []

    header = lines[header_index]
    delimiter = "," if "," in header else "\t" if "\t" in header else None
    if delimiter is None:
        return []

    reader = csv.DictReader(lines[header_index:], delimiter=delimiter)
    rows: list[dict[str, str]] = []
    for row in reader:
        normalized = {
            normalize_key(key): (value or "").strip()
            for key, value in row.items()
            if key is not None
        }
        if normalized:
            rows.append(normalized)
    return rows


def find_header_line(lines: list[str]) -> int | None:
    for index, line in enumerate(lines[:100]):
        normalized = line.strip().lower().replace(" ", "")
        if normalized.startswith("t_ms") or normalized.startswith("t_us"):
            return index
        if normalized.startswith("time_s") or normalized.startswith("t_s"):
            return index
        if "gx_parent" in normalized or "parent_gx" in normalized:
            return index
        if "imu" in normalized and "gx_dps" in normalized:
            return index
    return None


def normalize_key(key: str) -> str:
    return key.strip().lower().replace(" ", "_")


def has_any_gyro_column(keys: set[str]) -> bool:
    return any(key in keys for key in ("gx_dps", "gy_dps", "gz_dps", "gx", "gy", "gz"))


def parse_long_table_rows(rows: list[dict[str, str]]) -> list[ImuSample]:
    records: list[ImuSample] = []
    for row in rows:
        imu = row.get("imu", "")
        if not imu.upper().startswith("IMU"):
            continue
        try:
            records.append(
                ImuSample(
                    t_s=row_time_s(row),
                    imu=imu,
                    ax_mg=row_float(row, ("ax_mg", "ax")),
                    ay_mg=row_float(row, ("ay_mg", "ay")),
                    az_mg=row_float(row, ("az_mg", "az")),
                    gx_dps=row_float(row, ("gx_dps", "gx")),
                    gy_dps=row_float(row, ("gy_dps", "gy")),
                    gz_dps=row_float(row, ("gz_dps", "gz")),
                    **row_quat(row),
                )
            )
        except (KeyError, ValueError):
            continue
    return records


def parse_serial_text(text: str) -> list[ImuSample]:
    """Parse a serial log into samples, refusing logs with a timestamp reset.

    The MCU clock can wrap or restart within a long session (a concatenated
    recording, a mid-session reset). Parsing straight through such a log is
    worse than failing: downstream code keys samples by time, so a reset makes
    later samples collide with or reorder earlier ones, and the result is a
    plausible-looking recording whose synchronisation is silently wrong.

    Split the log first, then give each segment its own clock mapping.
    """
    records: list[ImuSample] = []
    previous_t_s: float | None = None
    for line in text.splitlines():
        parts = line.strip().split()
        # >= so a raw-only line and an SFLP line (... gz_dps qw qx qy qz) both parse.
        if len(parts) < len(SERIAL_COLUMNS) or not parts[1].upper().startswith("IMU"):
            continue
        try:
            t_s = float(parts[0]) / 1000.0
            if previous_t_s is not None and t_s < previous_t_s - 1.0:
                raise ValueError(
                    "IMU timestamp reset detected. Split the raw log first with split_imu_log.py; "
                    "each segment needs its own clock mapping."
                )
            previous_t_s = t_s
            quat = parse_trailing_quat(parts)
            records.append(
                ImuSample(
                    t_s=t_s,
                    imu=parts[1],
                    ax_mg=float(parts[4]),
                    ay_mg=float(parts[5]),
                    az_mg=float(parts[6]),
                    gx_dps=float(parts[7]),
                    gy_dps=float(parts[8]),
                    gz_dps=float(parts[9]),
                    qw=quat[0] if quat is not None else None,
                    qx=quat[1] if quat is not None else None,
                    qy=quat[2] if quat is not None else None,
                    qz=quat[3] if quat is not None else None,
                )
            )
        except ValueError as exc:
            # Malformed rows are skipped; a timestamp reset is not a malformed row
            # and must not be swallowed by the same handler.
            if "timestamp reset" in str(exc):
                raise
            continue
    return records


def pair_long_records(
    records: list[ImuSample],
    parent: str,
    child: str,
    max_pair_dt_s: float,
) -> list[PairSample]:
    if not records:
        return []

    parent_key = parent.upper()
    child_key = child.upper()
    by_time: dict[float, dict[str, ImuSample]] = {}
    for record in records:
        by_time.setdefault(record.t_s, {})[record.imu.upper()] = record

    exact_pairs: list[PairSample] = []
    for t_s in sorted(by_time):
        group = by_time[t_s]
        if parent_key in group and child_key in group:
            exact_pairs.append(make_pair(group[parent_key], group[child_key], t_s))
    if len(exact_pairs) >= 2:
        return normalize_pair_times(exact_pairs)

    parent_records = sorted((r for r in records if r.imu.upper() == parent_key), key=lambda r: r.t_s)
    child_records = sorted((r for r in records if r.imu.upper() == child_key), key=lambda r: r.t_s)
    nearest_pairs: list[PairSample] = []
    j = 0
    for parent_record in parent_records:
        while j + 1 < len(child_records) and child_records[j + 1].t_s <= parent_record.t_s:
            j += 1
        candidates = child_records[max(0, j - 1) : min(len(child_records), j + 2)]
        if not candidates:
            continue
        child_record = min(candidates, key=lambda c: abs(c.t_s - parent_record.t_s))
        if abs(child_record.t_s - parent_record.t_s) <= max_pair_dt_s:
            t_s = 0.5 * (parent_record.t_s + child_record.t_s)
            nearest_pairs.append(make_pair(parent_record, child_record, t_s))
    return normalize_pair_times(nearest_pairs)


def make_pair(parent_record: ImuSample, child_record: ImuSample, t_s: float) -> PairSample:
    return PairSample(
        t_s=t_s,
        parent_gyr_dps=(parent_record.gx_dps, parent_record.gy_dps, parent_record.gz_dps),
        child_gyr_dps=(child_record.gx_dps, child_record.gy_dps, child_record.gz_dps),
    )


def normalize_pair_times(pairs: list[PairSample]) -> list[PairSample]:
    if not pairs:
        return []
    pairs = sorted(pairs, key=lambda sample: sample.t_s)
    t0 = pairs[0].t_s
    return [
        PairSample(
            t_s=sample.t_s - t0,
            parent_gyr_dps=sample.parent_gyr_dps,
            child_gyr_dps=sample.child_gyr_dps,
        )
        for sample in pairs
    ]


def parse_wide_rows(rows: list[dict[str, str]], parent: str, child: str) -> list[PairSample]:
    pairs: list[PairSample] = []
    for row in rows:
        try:
            parent_gyr = row_gyro_vector(row, role="parent", imu_id=parent)
            child_gyr = row_gyro_vector(row, role="child", imu_id=child)
            pairs.append(PairSample(t_s=row_time_s(row), parent_gyr_dps=parent_gyr, child_gyr_dps=child_gyr))
        except (KeyError, ValueError):
            continue
    return normalize_pair_times(pairs)


def row_gyro_vector(row: dict[str, str], role: str, imu_id: str) -> tuple[float, float, float]:
    aliases = [role.lower(), normalize_key(imu_id)]
    candidate_sets: list[tuple[str, str, str]] = []
    for alias in aliases:
        candidate_sets.extend(
            [
                (f"gx_{alias}", f"gy_{alias}", f"gz_{alias}"),
                (f"{alias}_gx", f"{alias}_gy", f"{alias}_gz"),
                (f"gx_{alias}_dps", f"gy_{alias}_dps", f"gz_{alias}_dps"),
                (f"{alias}_gx_dps", f"{alias}_gy_dps", f"{alias}_gz_dps"),
            ]
        )
    for keys in candidate_sets:
        if all(key in row and row[key] != "" for key in keys):
            return tuple(float(row[key]) for key in keys)
    raise KeyError(f"Missing gyro vector for {role}/{imu_id}")


def row_time_s(row: dict[str, str]) -> float:
    if "t_s" in row and row["t_s"] != "":
        return float(row["t_s"])
    if "time_s" in row and row["time_s"] != "":
        return float(row["time_s"])
    if "t_ms" in row and row["t_ms"] != "":
        return float(row["t_ms"]) / 1000.0
    if "time_ms" in row and row["time_ms"] != "":
        return float(row["time_ms"]) / 1000.0
    for key in ("t_us", "time_us", "t_mcu_us", "timestamp_us"):
        if key in row and row[key] != "":
            return float(row[key]) / 1_000_000.0
    raise KeyError("Missing time column")


def row_float(row: dict[str, str], names: tuple[str, ...]) -> float:
    for name in names:
        if name in row and row[name] != "":
            return float(row[name])
    raise KeyError(names[0])


def row_quat(row: dict[str, str]) -> dict[str, float | None]:
    names = ("qw", "qx", "qy", "qz")
    if not all(name in row and row[name] != "" for name in names):
        return {}
    return {name: float(row[name]) for name in names}


def parse_trailing_quat(parts: list[str]) -> tuple[float, float, float, float] | None:
    if len(parts) < 14:
        return None
    try:
        return (float(parts[10]), float(parts[11]), float(parts[12]), float(parts[13]))
    except ValueError:
        return None


def integrate_twist(
    pairs: list[PairSample],
    axis: tuple[float, float, float],
    bias_seconds: float,
    tare_seconds: float,
) -> tuple[list[TwistSample], dict[str, object]]:
    bias_indices = window_indices(pairs, 0.0, bias_seconds)
    if not bias_indices:
        bias_indices = [0]

    parent_bias = mean_vector([pairs[index].parent_gyr_dps for index in bias_indices])
    child_bias = mean_vector([pairs[index].child_gyr_dps for index in bias_indices])

    raw_twist: list[float] = [0.0 for _ in pairs]
    rel_axis_values: list[float] = []
    calibrated_pairs: list[tuple[tuple[float, float, float], tuple[float, float, float]]] = []

    for sample in pairs:
        parent_cal = vector_sub(sample.parent_gyr_dps, parent_bias)
        child_cal = vector_sub(sample.child_gyr_dps, child_bias)
        rel_axis = dot(vector_sub(child_cal, parent_cal), axis)
        rel_axis_values.append(rel_axis)
        calibrated_pairs.append((parent_cal, child_cal))

    median_dt = median_sample_dt(pairs)
    for index in range(1, len(pairs)):
        dt = pairs[index].t_s - pairs[index - 1].t_s
        if dt <= 0.0:
            dt = median_dt
        raw_twist[index] = raw_twist[index - 1] + 0.5 * (rel_axis_values[index - 1] + rel_axis_values[index]) * dt

    tare_indices = window_indices_from_samples(pairs, 0.0, tare_seconds)
    tare_offset = mean([raw_twist[index] for index in tare_indices]) if tare_indices else raw_twist[0]

    samples: list[TwistSample] = []
    for index, pair in enumerate(pairs):
        parent_cal, child_cal = calibrated_pairs[index]
        samples.append(
            TwistSample(
                t_s=pair.t_s,
                parent_gyr_dps=pair.parent_gyr_dps,
                child_gyr_dps=pair.child_gyr_dps,
                parent_gyr_cal_dps=parent_cal,
                child_gyr_cal_dps=child_cal,
                rel_axis_dps=rel_axis_values[index],
                twist_deg=raw_twist[index] - tare_offset,
            )
        )

    detail = {
        "parent_bias_dps": parent_bias,
        "child_bias_dps": child_bias,
        "bias_sample_count": len(bias_indices),
        "tare_sample_count": len(tare_indices),
        "tare_offset_deg": tare_offset,
    }
    return samples, detail


def window_indices(pairs: list[PairSample], start_s: float, end_s: float) -> list[int]:
    return [index for index, sample in enumerate(pairs) if start_s <= sample.t_s <= end_s]


def window_indices_from_samples(pairs: list[PairSample], start_s: float, end_s: float) -> list[int]:
    return window_indices(pairs, start_s, end_s)


def summarize(
    samples: list[TwistSample],
    detail: dict[str, object],
    parent: str,
    child: str,
    axis: tuple[float, float, float],
    drift_start_s: float | None,
    drift_end_s: float | None,
    return_window_seconds: float,
) -> dict[str, object]:
    duration_s = samples[-1].t_s - samples[0].t_s
    dt_values = [
        samples[index].t_s - samples[index - 1].t_s
        for index in range(1, len(samples))
        if samples[index].t_s > samples[index - 1].t_s
    ]
    median_dt = statistics.median(dt_values) if dt_values else 0.0
    sample_rate_hz = 1.0 / median_dt if median_dt > 0 else None

    if drift_start_s is None and drift_end_s is None:
        drift_start = None
        drift_end = None
        drift_deg_per_min = None
    else:
        drift_start = 0.0 if drift_start_s is None else drift_start_s
        drift_end = samples[-1].t_s if drift_end_s is None else drift_end_s
        drift_window = [sample for sample in samples if drift_start <= sample.t_s <= drift_end]
        drift_deg_per_min = linear_slope_deg_per_min(drift_window)

    return_start = max(samples[0].t_s, samples[-1].t_s - max(return_window_seconds, 0.0))
    return_window = [sample.twist_deg for sample in samples if sample.t_s >= return_start]
    return_to_zero_deg = mean(return_window) if return_window else samples[-1].twist_deg

    twist_values = [sample.twist_deg for sample in samples]
    summary: dict[str, object] = {
        "algorithm": "gyro_only_relative_twist_v0",
        "parent": parent,
        "child": child,
        "axis": axis,
        "sample_count": len(samples),
        "duration_s": duration_s,
        "sample_rate_hz_median": sample_rate_hz,
        "twist_min_deg": min(twist_values),
        "twist_max_deg": max(twist_values),
        "twist_final_deg": samples[-1].twist_deg,
        "return_window_s": return_window_seconds,
        "return_to_zero_estimate_deg": return_to_zero_deg,
        "drift_fit_start_s": drift_start,
        "drift_fit_end_s": drift_end,
        "drift_fit_deg_per_min": drift_deg_per_min,
    }
    summary.update(detail)
    return summary


def linear_slope_deg_per_min(samples: list[TwistSample]) -> float | None:
    if len(samples) < 2:
        return None
    xs = [sample.t_s for sample in samples]
    ys = [sample.twist_deg for sample in samples]
    x_mean = mean(xs)
    y_mean = mean(ys)
    denom = sum((x - x_mean) ** 2 for x in xs)
    if denom <= 1e-12:
        return None
    slope_deg_per_s = sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys)) / denom
    return slope_deg_per_s * 60.0


def write_twist_csv(samples: list[TwistSample], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "t_s",
                "gx_parent_dps",
                "gy_parent_dps",
                "gz_parent_dps",
                "gx_child_dps",
                "gy_child_dps",
                "gz_child_dps",
                "gx_parent_cal_dps",
                "gy_parent_cal_dps",
                "gz_parent_cal_dps",
                "gx_child_cal_dps",
                "gy_child_cal_dps",
                "gz_child_cal_dps",
                "rel_axis_dps",
                "twist_deg",
            ]
        )
        for sample in samples:
            writer.writerow(
                [
                    f"{sample.t_s:.6f}",
                    *format_vector(sample.parent_gyr_dps),
                    *format_vector(sample.child_gyr_dps),
                    *format_vector(sample.parent_gyr_cal_dps),
                    *format_vector(sample.child_gyr_cal_dps),
                    f"{sample.rel_axis_dps:.6f}",
                    f"{sample.twist_deg:.6f}",
                ]
            )


def write_json(payload: dict[str, object], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(to_jsonable(payload), indent=2) + "\n", encoding="utf-8")


def to_jsonable(value: object) -> object:
    if isinstance(value, tuple):
        return list(value)
    if isinstance(value, dict):
        return {key: to_jsonable(item) for key, item in value.items()}
    return value


def write_svg_plot(samples: list[TwistSample], path: Path, title: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    width, height = 960, 520
    left, right, top, bottom = 76, 32, 50, 70
    plot_w = width - left - right
    plot_h = height - top - bottom

    xs = [sample.t_s for sample in samples]
    ys = [sample.twist_deg for sample in samples]
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(min(ys), 0.0), max(max(ys), 0.0)
    if math.isclose(x_min, x_max):
        x_max = x_min + 1.0
    if math.isclose(y_min, y_max):
        y_min -= 1.0
        y_max += 1.0
    y_pad = max((y_max - y_min) * 0.12, 1.0)
    y_min -= y_pad
    y_max += y_pad

    def x_px(x: float) -> float:
        return left + (x - x_min) / (x_max - x_min) * plot_w

    def y_px(y: float) -> float:
        return top + (y_max - y) / (y_max - y_min) * plot_h

    points = " ".join(f"{x_px(x):.2f},{y_px(y):.2f}" for x, y in zip(xs, ys))
    grid = []
    for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
        gx = left + frac * plot_w
        gy = top + frac * plot_h
        grid.append(f'<line x1="{gx:.2f}" y1="{top}" x2="{gx:.2f}" y2="{top + plot_h}" class="grid"/>')
        grid.append(f'<line x1="{left}" y1="{gy:.2f}" x2="{left + plot_w}" y2="{gy:.2f}" class="grid"/>')

    zero_line = ""
    if y_min <= 0.0 <= y_max:
        zero_y = y_px(0.0)
        zero_line = f'<line x1="{left}" y1="{zero_y:.2f}" x2="{left + plot_w}" y2="{zero_y:.2f}" class="zero"/>'

    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <style>
    text {{ font-family: Segoe UI, Arial, sans-serif; fill: #202124; }}
    .grid {{ stroke: #d8dde5; stroke-width: 1; }}
    .axis {{ stroke: #202124; stroke-width: 1.5; }}
    .zero {{ stroke: #7a8799; stroke-width: 1.2; stroke-dasharray: 5 5; }}
    .line {{ fill: none; stroke: #1864ab; stroke-width: 2.5; }}
    .label {{ font-size: 16px; }}
    .small {{ font-size: 13px; fill: #5f6368; }}
    .title {{ font-size: 20px; font-weight: 700; }}
  </style>
  <rect width="100%" height="100%" fill="#ffffff"/>
  <text x="{left}" y="30" class="title">{escape_xml(title)}</text>
  {''.join(grid)}
  {zero_line}
  <line x1="{left}" y1="{top}" x2="{left}" y2="{top + plot_h}" class="axis"/>
  <line x1="{left}" y1="{top + plot_h}" x2="{left + plot_w}" y2="{top + plot_h}" class="axis"/>
  <polyline points="{points}" class="line"/>
  <text x="{left + plot_w / 2:.1f}" y="{height - 22}" text-anchor="middle" class="label">time_s</text>
  <text x="24" y="{top + plot_h / 2:.1f}" transform="rotate(-90 24 {top + plot_h / 2:.1f})" text-anchor="middle" class="label">twist_deg</text>
  <text x="{left}" y="{top + plot_h + 24}" class="small">{x_min:.2f}s</text>
  <text x="{left + plot_w}" y="{top + plot_h + 24}" text-anchor="end" class="small">{x_max:.2f}s</text>
  <text x="{left - 12}" y="{top + 5}" text-anchor="end" class="small">{y_max:.1f}</text>
  <text x="{left - 12}" y="{top + plot_h}" text-anchor="end" class="small">{y_min:.1f}</text>
</svg>
"""
    path.write_text(svg, encoding="utf-8")


def print_summary(summary: dict[str, object], out_csv: Path, out_plot: Path, out_summary: Path) -> None:
    def fmt(value: object, digits: int = 3) -> str:
        if value is None:
            return "n/a"
        if isinstance(value, float):
            return f"{value:.{digits}f}"
        return str(value)

    print("SpineSense twist_bench_v0 complete")
    print(f"  parent -> child: {summary['parent']} -> {summary['child']}")
    print(f"  samples: {summary['sample_count']}, duration_s: {fmt(summary['duration_s'])}")
    print(f"  median_rate_hz: {fmt(summary['sample_rate_hz_median'])}")
    print(f"  twist range deg: {fmt(summary['twist_min_deg'])} to {fmt(summary['twist_max_deg'])}")
    print(f"  final twist deg: {fmt(summary['twist_final_deg'])}")
    print(f"  return-to-zero estimate deg: {fmt(summary['return_to_zero_estimate_deg'])}")
    print(f"  drift fit deg/min: {fmt(summary['drift_fit_deg_per_min'])}")
    print(f"  csv: {out_csv}")
    print(f"  plot: {out_plot}")
    print(f"  summary: {out_summary}")


def make_demo_pairs(axis: tuple[float, float, float], duration_s: float = 28.0, rate_hz: float = 120.0) -> list[PairSample]:
    del axis
    count = int(duration_s * rate_hz) + 1
    dt = 1.0 / rate_hz
    parent_bias = (0.04, -0.03, 0.11)
    child_bias = (-0.02, 0.02, 0.18)
    pairs: list[PairSample] = []
    previous_angle = demo_angle_deg(0.0)
    for index in range(count):
        t_s = index * dt
        angle = demo_angle_deg(t_s)
        omega_z = (angle - previous_angle) / dt if index else 0.0
        previous_angle = angle
        wobble = 0.08 * math.sin(2.0 * math.pi * 0.23 * t_s)
        parent = (
            parent_bias[0] + 0.03 * math.sin(0.7 * t_s),
            parent_bias[1] + 0.02 * math.cos(0.5 * t_s),
            parent_bias[2] + wobble,
        )
        child = (
            child_bias[0] + 0.02 * math.cos(0.6 * t_s),
            child_bias[1] + 0.03 * math.sin(0.4 * t_s),
            child_bias[2] + wobble + omega_z,
        )
        pairs.append(PairSample(t_s=t_s, parent_gyr_dps=parent, child_gyr_dps=child))
    return pairs


def demo_angle_deg(t_s: float) -> float:
    keyframes = [
        (0.0, 0.0),
        (5.0, 0.0),
        (8.0, 30.0),
        (12.0, 30.0),
        (15.0, 0.0),
        (17.0, 0.0),
        (20.0, -30.0),
        (23.0, -30.0),
        (26.0, 0.0),
        (28.0, 0.0),
    ]
    if t_s <= keyframes[0][0]:
        return keyframes[0][1]
    for (t0, y0), (t1, y1) in zip(keyframes, keyframes[1:]):
        if t0 <= t_s <= t1:
            frac = (t_s - t0) / (t1 - t0)
            smooth = frac * frac * (3.0 - 2.0 * frac)
            return y0 + (y1 - y0) * smooth
    return keyframes[-1][1]


def median_sample_dt(pairs: list[PairSample]) -> float:
    deltas = [
        pairs[index].t_s - pairs[index - 1].t_s
        for index in range(1, len(pairs))
        if pairs[index].t_s > pairs[index - 1].t_s
    ]
    return statistics.median(deltas) if deltas else 0.0


def mean(values: list[float]) -> float:
    return sum(values) / len(values)


def mean_vector(values: list[tuple[float, float, float]]) -> tuple[float, float, float]:
    return (
        mean([value[0] for value in values]),
        mean([value[1] for value in values]),
        mean([value[2] for value in values]),
    )


def vector_sub(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vector_norm(a: tuple[float, float, float]) -> float:
    return math.sqrt(dot(a, a))


def dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def format_vector(vector: tuple[float, float, float]) -> list[str]:
    return [f"{value:.6f}" for value in vector]


def escape_xml(text: str) -> str:
    return (
        text.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
        .replace("'", "&apos;")
    )


if __name__ == "__main__":
    sys.exit(main())
