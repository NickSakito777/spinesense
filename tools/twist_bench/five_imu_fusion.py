from __future__ import annotations

"""Five-IMU SpineSense fusion, v1.

This is the first body-chain step after the two-IMU twist bench:

raw 5-IMU accel/gyro log
-> still-window gyro bias subtraction
-> per-IMU 6D orientation filter
-> sensor-to-segment up-axis alignment
-> root-relative and adjacent relative quaternions
-> neutral tare
-> swing/twist angles for regional segment relations

The output is short-term, neutral-tared, relative regional orientation only.
It is not absolute yaw, not 3D position, and not per-vertebra rotation.
"""

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import twist_bench_fusion as pair_fusion
import twist_bench_v0 as v0


BODY5_LAYOUT = {
    "pelvis": "IMU0",
    "lower": "IMU1",
    "mid": "IMU2",
    "upper": "IMU3",
    "sternum": "IMU4",
}

SPINE5_U5_TOP_LAYOUT = {
    "bottom": "IMU0",  # board U1, lowest
    "low": "IMU1",     # board U2
    "mid": "IMU2",     # board U3
    "high": "IMU3",    # board U4
    "top": "IMU4",     # board U5, highest
}

VALIDATION1_LAYOUT = {
    "t3": "IMU0",       # upper thoracic posterior reference
    "t6": "IMU1",       # mid-thoracic posterior anchor
    "t12": "IMU2",      # thoracolumbar posterior reference
    "sacrum": "IMU3",   # pelvis / root reference
    "sternum": "IMU4",  # anterior thorax reference
}
# The Validation1 sid->role mapping above is the documented DEFAULT only. Firmware sid is bound to a
# fixed GPIO/TA0 socket, NOT to anatomy, so the authoritative binding must come from per-trial
# metadata and override this preset (see validation1_analysis.load_placement_map).

# Historical T01 mapping used by the 2026-07-03..07-07 analyses.  The 2026-07-13 audit found
# that this mapping reverses the posterior chain.  It is retained only to reproduce legacy
# artifacts and MUST NOT be used for production analysis without --allow-legacy-preset.
LEGACY_T01_LAYOUT = {
    "sacrum": "IMU4",
    "lower": "IMU3",
    "mid": "IMU2",
    "upper": "IMU1",
    "sternum": "IMU0",
}

# Anatomical five-role schema used with a mandatory per-trial placement registry.  These values
# are only a parser bootstrap: production callers must override all five roles and provide the
# registry provenance fields.  The current registry resolves the cohort to the reverse chain
# (IMU1=sacrum ... IMU4=upper, IMU0=sternum), but that decision lives in config, not here.
TRIAL5_SCHEMA_LAYOUT = dict(LEGACY_T01_LAYOUT)

LAYOUT_PRESETS = {
    "body5": BODY5_LAYOUT,
    "spine5-u5-top": SPINE5_U5_TOP_LAYOUT,
    "validation1": VALIDATION1_LAYOUT,
    "trial5": TRIAL5_SCHEMA_LAYOUT,
    "t01": LEGACY_T01_LAYOUT,
}

ROLE_ORDERS = {
    "body5": ["pelvis", "lower", "mid", "upper", "sternum"],
    "spine5-u5-top": ["bottom", "low", "mid", "high", "top"],
    "validation1": ["sacrum", "t12", "t6", "t3", "sternum"],
    "trial5": ["sacrum", "lower", "mid", "upper", "sternum"],
    "t01": ["sacrum", "lower", "mid", "upper", "sternum"],
}

RELATION_PRESETS = {
    "body5": [
        ("pelvis_to_lower", "pelvis", "lower"),
        ("lower_to_mid", "lower", "mid"),
        ("mid_to_upper", "mid", "upper"),
        ("pelvis_to_upper", "pelvis", "upper"),
        ("pelvis_to_sternum", "pelvis", "sternum"),
        ("sternum_to_upper_check", "sternum", "upper"),
    ],
    "spine5-u5-top": [
        ("bottom_to_low", "bottom", "low"),
        ("low_to_mid", "low", "mid"),
        ("mid_to_high", "mid", "high"),
        ("high_to_top", "high", "top"),
        ("bottom_to_top", "bottom", "top"),
    ],
    # Validation1 posterior chain + sternum cross-check. parent = inferior segment, so
    # q_rel = inverse(parent) * child expresses the superior/anterior segment in the inferior frame.
    "validation1": [
        # posterior adjacent chain (sacrum -> T12 -> T6 -> T3)
        ("sacrum_to_t12", "sacrum", "t12"),
        ("t12_to_t6", "t12", "t6"),
        ("t6_to_t3", "t6", "t3"),
        # composite total-back relation (~= product of the adjacent chain; a redundant cross-check,
        # do NOT pool it with the adjacent chain in any aggregate).
        ("sacrum_to_t3", "sacrum", "t3"),
        # sternum is an anterior reference: cross-check + features, never an automatic correction.
        ("sacrum_to_sternum", "sacrum", "sternum"),
        ("t3_to_sternum", "t3", "sternum"),
        ("t6_to_sternum", "t6", "sternum"),
    ],
    # Trial posterior chain (sacrum root -> lower -> mid -> upper) + anterior sternum cross-check.
    # parent = inferior segment, so q_rel = inverse(parent) * child expresses the superior segment.
    "trial5": [
        ("sacrum_to_lower", "sacrum", "lower"),
        ("lower_to_mid", "lower", "mid"),
        ("mid_to_upper", "mid", "upper"),
        # composite total-back relation (~= product of the adjacent chain; redundant cross-check,
        # do NOT pool it with the adjacent chain in any aggregate).
        ("sacrum_to_upper", "sacrum", "upper"),
        # sternum anterior reference: twist channel + cross-check, never an automatic correction.
        ("sacrum_to_sternum", "sacrum", "sternum"),
        ("upper_to_sternum", "upper", "sternum"),
        ("mid_to_sternum", "mid", "sternum"),
    ],
}

# Preserve the relation schema solely for exact legacy-result reproduction.
RELATION_PRESETS["t01"] = list(RELATION_PRESETS["trial5"])

# Anterior/posterior sternum cross-check per layout: compare the anterior (sternum-vs-root) tared
# relation against the posterior (top-of-back-vs-root) tared relation. Large disagreement means the
# anterior and posterior references diverge. This is a CHECK, not a correction. Layouts without an
# entry compute no cross-check.
CROSS_CHECKS = {
    "body5": ("pelvis_to_upper", "pelvis_to_sternum"),
    "validation1": ("sacrum_to_t3", "sacrum_to_sternum"),
    "trial5": ("sacrum_to_upper", "sacrum_to_sternum"),
    "t01": ("sacrum_to_upper", "sacrum_to_sternum"),
}


@dataclass(frozen=True)
class ImuStream:
    imu: str
    acc_mg: np.ndarray
    gyr_dps: np.ndarray
    q_sflp: np.ndarray | None = None


@dataclass
class SensorState:
    role: str
    imu: str
    acc_mg: np.ndarray
    gyr_raw_dps: np.ndarray
    gyr_cal_dps: np.ndarray
    gyro_bias_dps: np.ndarray
    up_sensor: np.ndarray
    q_filter: np.ndarray
    q_segment: np.ndarray
    filter_info: dict[str, object]
    rest_detected: np.ndarray | None


@dataclass
class RelationResult:
    name: str
    parent_role: str
    child_role: str
    q_rel: np.ndarray
    q_tared: np.ndarray
    twist_deg: np.ndarray
    swing_deg: np.ndarray
    summary: dict[str, object]


@dataclass
class FiveImuResult:
    t_s: np.ndarray
    sensors: dict[str, SensorState]
    relations: dict[str, RelationResult]
    summary: dict[str, object]


def load_five_streams(path: Path, layout: dict[str, str]) -> tuple[np.ndarray, dict[str, ImuStream]]:
    text = path.read_text(encoding="utf-8", errors="replace")
    records = v0.parse_serial_text(text)
    if not records:
        rows = v0.read_dict_rows(text)
        records = v0.parse_long_table_rows(rows)
    if not records:
        raise SystemExit(f"No IMU records parsed from {path}")

    imu_ids = {imu.upper() for imu in layout.values()}
    by_time: dict[float, dict[str, v0.ImuSample]] = {}
    for record in records:
        imu = record.imu.upper()
        if imu in imu_ids:
            by_time.setdefault(record.t_s, {})[imu] = record

    if not by_time:
        raise SystemExit(f"No selected IMUs found. Requested: {sorted(imu_ids)}")

    rows_t: list[float] = []
    acc: dict[str, list[tuple[float, float, float]]] = {imu: [] for imu in imu_ids}
    gyr: dict[str, list[tuple[float, float, float]]] = {imu: [] for imu in imu_ids}
    quat: dict[str, list[tuple[float, float, float, float] | None]] = {imu: [] for imu in imu_ids}
    for t_s in sorted(by_time):
        group = by_time[t_s]
        if all(imu in group for imu in imu_ids):
            rows_t.append(t_s)
            for imu in imu_ids:
                sample = group[imu]
                acc[imu].append((sample.ax_mg, sample.ay_mg, sample.az_mg))
                gyr[imu].append((sample.gx_dps, sample.gy_dps, sample.gz_dps))
                quat[imu].append(sample.sflp_quat)

    if len(rows_t) < 10:
        present = sorted({imu for group in by_time.values() for imu in group})
        raise SystemExit(
            f"Only {len(rows_t)} complete 5-IMU frames found; need more data. "
            f"Present selected IDs: {present}"
        )

    t = np.asarray(rows_t, dtype=float)
    t = t - t[0]
    streams: dict[str, ImuStream] = {}
    for imu in sorted(imu_ids):
        q_sflp = None
        if quat[imu] and all(q is not None for q in quat[imu]):
            try:
                q_sflp = normalize_quat_series(np.asarray(quat[imu], dtype=float))
            except SystemExit:
                q_sflp = None
        streams[imu] = ImuStream(
            imu=imu,
            acc_mg=np.asarray(acc[imu], dtype=float),
            gyr_dps=np.asarray(gyr[imu], dtype=float),
            q_sflp=q_sflp,
        )
    return t, streams


def normalize_quat_series(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    if q.ndim != 2 or q.shape[1] != 4 or not np.all(np.isfinite(q)):
        raise SystemExit("SFLP quaternion stream must be finite Nx4 [w,x,y,z].")
    norms = np.linalg.norm(q, axis=1)
    if np.any(norms < 1e-9):
        raise SystemExit("SFLP quaternion stream contains a zero-norm row.")
    q = q / norms[:, None]
    for i in range(1, len(q)):
        if float(np.dot(q[i - 1], q[i])) < 0.0:
            q[i] = -q[i]
    return q


def parse_layout(args: argparse.Namespace) -> dict[str, str]:
    layout = dict(LAYOUT_PRESETS[args.layout_preset])
    explicit_roles = {role: getattr(args, role) for role in layout if getattr(args, role)}

    if args.layout_preset == "t01" and not args.allow_legacy_preset:
        raise SystemExit(
            "The t01 preset is the quarantined legacy mapping (posterior chain reversed). "
            "Use layout_preset='trial5' with a resolved per-trial placement map. "
            "Pass --allow-legacy-preset only for an explicitly labelled legacy reproduction."
        )

    if args.layout_preset == "trial5" and not args.demo:
        missing_roles = sorted(set(layout) - set(explicit_roles))
        provenance = {
            "trial_id": args.trial_id,
            "placement_map_source": args.placement_map_source,
            "mapping_version": args.mapping_version,
            "mapping_status": args.mapping_status,
            "mapping_sha256": args.mapping_sha256,
        }
        missing_provenance = sorted(key for key, value in provenance.items() if not value)
        if missing_roles or missing_provenance:
            raise SystemExit(
                "trial5 production analysis requires a complete per-trial placement map and provenance; "
                f"missing_roles={missing_roles}, missing_provenance={missing_provenance}. "
                "Resolve the trial through placement_maps.resolve_placement(...)."
            )
        if args.mapping_status not in {"confirmed", "inferred_high"}:
            raise SystemExit(
                f"trial5 mapping status {args.mapping_status!r} is not production-eligible; "
                "expected 'confirmed' or 'inferred_high'."
            )
        digest = str(args.mapping_sha256).lower()
        if len(digest) != 64 or any(ch not in "0123456789abcdef" for ch in digest):
            raise SystemExit("mapping_sha256 must be a 64-character lowercase hexadecimal SHA256 digest.")

    for role in layout:
        value = getattr(args, role)
        if value:
            layout[role] = value.upper()
    if len(set(layout.values())) != len(layout):
        raise SystemExit(f"Layout maps multiple roles to the same IMU: {layout}")
    if args.layout_preset == "trial5" and set(layout.values()) != {f"IMU{i}" for i in range(5)}:
        raise SystemExit(f"trial5 placement must be a bijection over IMU0..IMU4: {layout}")
    return layout


def choose_window(t: np.ndarray, start_s: float | None, end_s: float | None, fallback_seconds: float) -> np.ndarray:
    if start_s is not None or end_s is not None:
        lo = 0.0 if start_s is None else float(start_s)
        hi = float(t[-1]) if end_s is None else float(end_s)
        mask = (t >= lo) & (t <= hi)
    else:
        mask = t <= float(fallback_seconds)
    return mask


def derive_windows_from_markers(input_path: Path) -> dict[str, float]:
    marker_path = input_path.with_name(input_path.stem + "_markers.json")
    if not marker_path.exists():
        return {}
    try:
        markers = json.loads(marker_path.read_text(encoding="utf-8"))
    except Exception:
        return {}

    out: dict[str, float] = {}
    for phase in markers.get("phases", []):
        name = phase.get("name")
        if name == "neutral":
            out["bias_start_s"] = float(phase["start_s"])
            out["bias_end_s"] = float(phase["end_s"])
            out["tare_start_s"] = float(phase["start_s"])
            out["tare_end_s"] = float(phase["end_s"])
        elif name == "return_neutral":
            out["end_still_start_s"] = max(float(phase["start_s"]), float(phase["end_s"]) - 4.0)
            out["end_still_end_s"] = float(phase["end_s"])
        elif name == "static":
            # Static-hold recording (GUI Static Hold mode): use the first 8 s as bias/tare,
            # the remainder as the drift window (the held span the segments should keep flat).
            start = float(phase["start_s"])
            end = float(phase["end_s"])
            bias_end = min(start + 8.0, end)
            out["bias_start_s"] = start
            out["bias_end_s"] = bias_end
            out["tare_start_s"] = start
            out["tare_end_s"] = bias_end
            out["drift_start_s"] = bias_end
            out["drift_end_s"] = end
    return out


def resolve_up(name: str, acc_mg: np.ndarray, still: np.ndarray) -> np.ndarray:
    return pair_fusion.resolve_up(name, acc_mg, still)


def run_sensor_filter(
    role: str,
    stream: ImuStream,
    t: np.ndarray,
    still: np.ndarray,
    filter_name: str,
    up_name: str,
) -> SensorState:
    dts = np.diff(t)
    ts_med = float(np.median(dts))
    gyro_bias = stream.gyr_dps[still].mean(axis=0)
    gyr_cal = stream.gyr_dps - gyro_bias

    if filter_name == "sflp":
        if stream.q_sflp is None:
            raise SystemExit(f"--filter sflp requested, but {stream.imu} has no complete qw/qx/qy/qz stream.")
        q_filter = stream.q_sflp
        info = {"source": "on_chip_sflp", "quat_convention": "[w,x,y,z], sensor_to_world"}
        rest = None
    elif filter_name == "vqf":
        q_filter, info, rest = pair_fusion.run_vqf(gyr_cal, stream.acc_mg, ts_med)
    else:
        q_filter, info, rest = pair_fusion.run_madgwick(gyr_cal, stream.acc_mg, t)

    up_sensor = resolve_up(up_name, stream.acc_mg, still)
    q_align = pair_fusion.quat_align_z_to(up_sensor)
    q_segment = pair_fusion.qmul(q_filter, q_align)
    return SensorState(
        role=role,
        imu=stream.imu,
        acc_mg=stream.acc_mg,
        gyr_raw_dps=stream.gyr_dps,
        gyr_cal_dps=gyr_cal,
        gyro_bias_dps=gyro_bias,
        up_sensor=up_sensor,
        q_filter=q_filter,
        q_segment=q_segment,
        filter_info=info,
        rest_detected=rest,
    )


def relation_metrics(
    name: str,
    parent: SensorState,
    child: SensorState,
    t: np.ndarray,
    tare: np.ndarray,
    end_still: np.ndarray,
    drift: np.ndarray,
    twist_axis_mode: str = "fixed",
    calib_mask: np.ndarray | None = None,
) -> RelationResult:
    q_rel = pair_fusion.qmul(pair_fusion.qconj(parent.q_segment), child.q_segment)
    q_rel0 = pair_fusion.quat_average(q_rel[tare])
    q_tared = pair_fusion.qmul(pair_fusion.qconj(q_rel0)[None, :], q_rel)
    # Twist decomposition axis. 'fixed' = world-Z [0,0,1] (neutral longitudinal axis), the original
    # behaviour. 'functional' = a per-relation flexion-decoupled axis from sensor-to-segment
    # calibration, which removes the tan(theta/2) flexion->twist cross-talk (see estimate_twist_axis).
    if twist_axis_mode == "functional" and calib_mask is not None:
        twist_axis = pair_fusion.estimate_twist_axis(q_tared, calib_mask)
    else:
        twist_axis = pair_fusion.SEGMENT_TWIST_AXIS
    twist_deg, swing_deg = pair_fusion.swing_twist_deg(q_tared, twist_axis)
    twist_deg = pair_fusion.unwrap_deg(twist_deg)

    end_mean = float(np.mean(twist_deg[end_still])) if np.any(end_still) else float(twist_deg[-1])
    end_rms = float(np.sqrt(np.mean((twist_deg[end_still] - end_mean) ** 2))) if np.any(end_still) else None
    tare_rms = float(np.sqrt(np.mean((twist_deg[tare] - np.mean(twist_deg[tare])) ** 2)))
    # Static drift: linear slope of tared twist over the drift window, in deg/min. The true value
    # is flat (segments are not moving), so a non-zero slope is residual bias/heading drift. Same
    # polyfit convention as batch_analyze.end_still_drift_deg_per_min (2-IMU). None if too few samples.
    static_drift = None
    if int(np.count_nonzero(drift)) >= 5:
        static_drift = float(np.polyfit(t[drift], twist_deg[drift], 1)[0]) * 60.0
    # Peak timing: when the twist reaches its largest excursion. Used to compare temporal alignment
    # across segments (and later across SFLP vs VQF). Reported for the absolute peak plus the signed
    # positive/negative peaks so a movement can be timed in either direction.
    abs_idx = int(np.argmax(np.abs(twist_deg)))
    pos_idx = int(np.argmax(twist_deg))
    neg_idx = int(np.argmin(twist_deg))
    swing_idx = int(np.argmax(swing_deg))
    summary = {
        "parent_role": parent.role,
        "child_role": child.role,
        "parent_imu": parent.imu,
        "child_imu": child.imu,
        "twist_axis_mode": twist_axis_mode,
        "twist_axis": [round(float(v), 4) for v in twist_axis],
        "twist_min_deg": float(np.min(twist_deg)),
        "twist_max_deg": float(np.max(twist_deg)),
        "twist_range_deg": float(np.max(twist_deg) - np.min(twist_deg)),
        "twist_final_deg": float(twist_deg[-1]),
        "twist_abs_peak_deg": float(twist_deg[abs_idx]),
        "twist_abs_peak_time_s": float(t[abs_idx]),
        "twist_pos_peak_time_s": float(t[pos_idx]),
        "twist_neg_peak_time_s": float(t[neg_idx]),
        "return_to_zero_estimate_deg": end_mean,
        "end_still_twist_rms_deg": end_rms,
        "tare_window_twist_rms_deg": tare_rms,
        "static_drift_deg_per_min": static_drift,
        "swing_max_deg": float(np.max(swing_deg)),
        "swing_peak_time_s": float(t[swing_idx]),
    }
    return RelationResult(
        name=name,
        parent_role=parent.role,
        child_role=child.role,
        q_rel=q_rel,
        q_tared=q_tared,
        twist_deg=twist_deg,
        swing_deg=swing_deg,
        summary=summary,
    )


def angle_between_quats_deg(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    q_delta = pair_fusion.qmul(pair_fusion.qconj(a), b)
    q_delta = pair_fusion.qnormalize(q_delta)
    return np.degrees(2.0 * np.arccos(np.clip(np.abs(q_delta[..., 0]), -1.0, 1.0)))


def run_pipeline(path: Path | None, args: argparse.Namespace) -> FiveImuResult:
    # Compatibility migration for the many 2026-07 cohort scripts that still spell the old
    # preset name.  This is NOT a preset fallback: the raw filename must resolve uniquely through
    # the strict per-trial registry, otherwise production stops.  Exact legacy reproduction must
    # opt in with allow_legacy_preset=True and is marked in provenance.
    if args.layout_preset == "t01" and not args.allow_legacy_preset and not args.demo:
        if path is None:
            raise SystemExit("Deprecated t01 call cannot be resolved without an input raw path.")
        try:
            import placement_maps as placement_registry

            placement = placement_registry.resolve_placement(raw_path=path)
        except Exception as exc:
            raise SystemExit(
                f"Deprecated t01 call could not resolve an eligible per-trial placement map for {path}: {exc}"
            ) from exc
        for key, value in placement.fusion_kwargs().items():
            setattr(args, key, value)
        setattr(args, "deprecated_t01_auto_resolved", True)

    if args.layout_preset == "trial5" and not args.demo:
        if path is None:
            raise SystemExit("trial5 production analysis requires an input raw path for registry cross-checking.")
        try:
            import placement_maps as placement_registry

            authoritative = placement_registry.resolve_placement(
                trial_id=args.trial_id,
                raw_path=path,
            )
        except Exception as exc:
            raise SystemExit(f"trial5 placement registry cross-check failed for {path}: {exc}") from exc
        expected = authoritative.fusion_kwargs()
        supplied = {
            key: (getattr(args, key).upper() if key in authoritative.role_to_imu and getattr(args, key) else getattr(args, key))
            for key in expected
        }
        mismatches = {
            key: {"supplied": supplied.get(key), "registry": value}
            for key, value in expected.items()
            if supplied.get(key) != value
        }
        if mismatches:
            raise SystemExit(
                "trial5 supplied placement/provenance does not match the authoritative registry entry: "
                f"{mismatches}"
            )

    layout = parse_layout(args)
    if args.demo:
        t, streams = make_demo_streams(layout)
        input_label = "demo"
    else:
        if path is None:
            raise SystemExit("Provide --input or use --demo.")
        marker_windows = derive_windows_from_markers(path) if args.auto_markers else {}
        for key, value in marker_windows.items():
            if getattr(args, key) is None:
                setattr(args, key, value)
        t, streams = load_five_streams(path, layout)
        input_label = str(path)

    if len(t) < 10:
        raise SystemExit("Need at least 10 synchronized samples.")
    dts = np.diff(t)
    still = choose_window(t, args.bias_start_s, args.bias_end_s, args.bias_seconds)
    tare = choose_window(t, args.tare_start_s, args.tare_end_s, args.tare_seconds)
    end_still = choose_window(t, args.end_still_start_s, args.end_still_end_s, args.return_window_seconds)
    if args.end_still_start_s is None and args.end_still_end_s is None:
        end_still = t >= (t[-1] - args.return_window_seconds)
    # Drift window: explicit --drift-start-s/--drift-end-s if given, else reuse the end-still window.
    # For a pure static-hold trial point this at the held span (e.g. after the bias/tare seconds).
    if args.drift_start_s is not None or args.drift_end_s is not None:
        drift = choose_window(t, args.drift_start_s, args.drift_end_s, args.return_window_seconds)
    else:
        drift = end_still

    if int(np.count_nonzero(still)) < 5:
        raise SystemExit("Bias/still window has fewer than 5 samples.")
    if int(np.count_nonzero(tare)) < 3:
        raise SystemExit("Tare window has fewer than 3 samples.")
    if np.any(tare & end_still):
        raise SystemExit("Tare window overlaps end-still window; return-to-zero would be circular.")

    # Functional twist-axis calibration window(s): pure-bend movement(s) that carry no axial twist.
    # Required when --twist-axis-mode functional; each relation fits its own axis on the union. Pair
    # flexion with lateral (or use a circumduction) so the longitudinal axis is fully determined.
    twist_calib = None
    if args.twist_axis_mode == "functional":
        if not args.twist_calib_window:
            raise SystemExit(
                "--twist-axis-mode functional needs at least one --twist-calib-window START END "
                "(pure-bend movement with no axial twist; flexion + lateral recommended)."
            )
        twist_calib = np.zeros(len(t), dtype=bool)
        for c0, c1 in args.twist_calib_window:
            twist_calib |= (t >= min(c0, c1)) & (t <= max(c0, c1))
        if int(np.count_nonzero(twist_calib)) < 10:
            raise SystemExit("Twist calibration window(s) have fewer than 10 samples total.")

    warnings: list[str] = []
    if getattr(args, "deprecated_t01_auto_resolved", False):
        warnings.append(
            "Deprecated t01 caller was auto-resolved through the per-trial placement registry; "
            "update the caller to request trial5 explicitly."
        )
    if args.allow_legacy_preset:
        warnings.append(
            "LEGACY REPRODUCTION ONLY: anatomically wrong t01 posterior mapping was explicitly enabled."
        )
    if int(np.sum(dts > 0.5)) > 0:
        warnings.append(f"{int(np.sum(dts > 0.5))} sample gaps > 0.5 s; fixed-Ts fusion may degrade.")

    sensors: dict[str, SensorState] = {}
    for role, imu in layout.items():
        up_name = args.up_axis
        role_up = getattr(args, f"{role}_up")
        if role_up:
            up_name = role_up
        stream = streams[imu.upper()]
        state = run_sensor_filter(role, stream, t, still, args.filter, up_name)
        sensors[role] = state
        gyro_std = float(np.max(stream.gyr_dps[still].std(axis=0)))
        if gyro_std > args.still_gyro_std_max:
            warnings.append(f"{role}/{imu} gyro std in bias window is {gyro_std:.2f} dps; window may contain motion.")

    relation_defs = RELATION_PRESETS[args.layout_preset]
    relations = {
        name: relation_metrics(
            name, sensors[parent_role], sensors[child_role], t, tare, end_still, drift,
            twist_axis_mode=args.twist_axis_mode, calib_mask=twist_calib,
        )
        for name, parent_role, child_role in relation_defs
    }

    summary = {
        "algorithm": f"five_imu_relative_orientation_v1_{args.filter}",
        "input": input_label,
        "layout_preset": args.layout_preset,
        "role_order": ROLE_ORDERS[args.layout_preset],
        "layout": layout,
        "placement_provenance": {
            "trial_id": args.trial_id,
            "placement_map_source": args.placement_map_source,
            "mapping_version": args.mapping_version,
            "mapping_status": args.mapping_status,
            "mapping_sha256": args.mapping_sha256,
            "legacy_preset_allowed": bool(args.allow_legacy_preset),
            "deprecated_t01_auto_resolved": bool(getattr(args, "deprecated_t01_auto_resolved", False)),
        },
        "relations": {name: rel.summary for name, rel in relations.items()},
        "sample_count": int(len(t)),
        "duration_s": float(t[-1]),
        "sample_rate_hz_median": float(1.0 / np.median(dts)),
        "sample_dt_ms_p5_p95": [float(np.percentile(dts, 5) * 1000.0), float(np.percentile(dts, 95) * 1000.0)],
        "bias_window_s": [float(t[still][0]), float(t[still][-1])],
        "tare_window_s": [float(t[tare][0]), float(t[tare][-1])],
        "end_still_window_s": [float(t[end_still][0]), float(t[end_still][-1])] if np.any(end_still) else None,
        "drift_window_s": [float(t[drift][0]), float(t[drift][-1])] if np.any(drift) else None,
        "sensors": {
            role: {
                "imu": state.imu,
                "up_sensor": [round(float(v), 4) for v in state.up_sensor],
                "gyro_bias_dps": [round(float(v), 4) for v in state.gyro_bias_dps],
                "sflp_available": state.filter_info.get("source") == "on_chip_sflp" or streams[state.imu].q_sflp is not None,
                "filter_info": state.filter_info,
            }
            for role, state in sensors.items()
        },
        "claim_boundary": (
            "Outputs are short-term neutral-tared relative regional orientations from magnetometer-free 6-axis IMUs; "
            "not absolute yaw, not 3D position, not per-vertebra rotation."
        ),
    }
    cross_check = CROSS_CHECKS.get(args.layout_preset)
    if cross_check is not None and all(name in relations for name in cross_check):
        posterior_name, anterior_name = cross_check
        posterior_vs_root = relations[posterior_name].q_tared
        anterior_vs_root = relations[anterior_name].q_tared
        anterior_posterior_delta = angle_between_quats_deg(posterior_vs_root, anterior_vs_root)
        summary["sternum_cross_check"] = {
            "posterior_relation": posterior_name,
            "anterior_relation": anterior_name,
            "angle_delta_mean_deg": float(np.mean(anterior_posterior_delta)),
            "angle_delta_max_deg": float(np.max(anterior_posterior_delta)),
            "angle_delta_end_still_mean_deg": float(np.mean(anterior_posterior_delta[end_still])) if np.any(end_still) else None,
            "note": "Large values mean the posterior (back) and anterior (sternum) references disagree; this is a check, not a correction.",
        }
    else:
        summary["cross_check_note"] = (
            "No anterior/posterior sternum cross-check is defined for this layout; "
            "all relations are treated as a single chain."
        )
    if warnings:
        summary["warnings"] = warnings
    return FiveImuResult(t_s=t, sensors=sensors, relations=relations, summary=summary)


def write_outputs(result: FiveImuResult, stem: str, args: argparse.Namespace) -> tuple[Path, Path, Path]:
    plots_dir = Path(__file__).resolve().parent / "plots"
    plots_dir.mkdir(exist_ok=True)
    out_csv = args.out_csv or plots_dir / f"{stem}_five_imu_{args.filter}.csv"
    out_svg = args.plot or plots_dir / f"{stem}_five_imu_{args.filter}.svg"
    out_json = args.summary or plots_dir / f"{stem}_five_imu_{args.filter}_summary.json"

    header = ["t_s"]
    role_order = result.summary["role_order"]
    for role in role_order:
        header += [f"q{ax}_{role}" for ax in "wxyz"]
    for name in result.relations:
        header += [f"{name}_twist_deg", f"{name}_swing_deg"]

    with out_csv.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(header)
        for i in range(len(result.t_s)):
            row: list[str] = [f"{result.t_s[i]:.4f}"]
            for role in role_order:
                row += [f"{value:.6f}" for value in result.sensors[role].q_segment[i]]
            for rel in result.relations.values():
                row += [f"{rel.twist_deg[i]:.4f}", f"{rel.swing_deg[i]:.4f}"]
            writer.writerow(row)

    write_svg(result, out_svg, stem)
    with out_json.open("w", encoding="utf-8") as fh:
        json.dump(result.summary, fh, indent=2)
        fh.write("\n")
    return out_csv, out_svg, out_json


def write_svg(result: FiveImuResult, path: Path, stem: str) -> None:
    width, height, pad = 1080, 520, 64
    colors = ["#0a6cff", "#d0342c", "#3aa655", "#8e44ad", "#d9822b", "#546a7b"]
    series = [
        (rel.twist_deg, colors[index % len(colors)], f"{name} twist")
        for index, (name, rel) in enumerate(result.relations.items())
    ]
    t = result.t_s
    all_y = np.concatenate([values for values, _, _ in series])
    y_min, y_max = float(np.min(all_y)), float(np.max(all_y))
    if y_max - y_min < 1.0:
        y_min -= 1.0
        y_max += 1.0
    y_pad = max(1.0, (y_max - y_min) * 0.08)
    y_min -= y_pad
    y_max += y_pad

    def sx(x: float) -> float:
        return pad + (x - t[0]) / max(1e-9, t[-1] - t[0]) * (width - 2 * pad)

    def sy(y: float) -> float:
        return height - pad - (y - y_min) / (y_max - y_min) * (height - 2 * pad)

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        f'<rect width="{width}" height="{height}" fill="white"/>',
        f'<text x="{pad}" y="28" font-family="sans-serif" font-size="16">{stem}: five-IMU tared relative twist (deg)</text>',
    ]
    if y_min < 0 < y_max:
        zy = sy(0.0)
        parts.append(f'<line x1="{pad}" y1="{zy:.1f}" x2="{width - pad}" y2="{zy:.1f}" stroke="#999" stroke-dasharray="4 4"/>')
    for index, (values, color, label) in enumerate(series):
        pts = " ".join(f"{sx(float(x)):.1f},{sy(float(y)):.1f}" for x, y in zip(t, values))
        parts.append(f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="1.5"/>')
        parts.append(
            f'<text x="{width - pad - 260}" y="{52 + index * 18}" font-family="sans-serif" font-size="13" fill="{color}">{label}</text>'
        )
    for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
        xv = t[0] + frac * (t[-1] - t[0])
        yv = y_min + frac * (y_max - y_min)
        parts.append(f'<text x="{sx(float(xv)):.1f}" y="{height - pad + 20}" font-family="sans-serif" font-size="11" text-anchor="middle">{xv:.0f}s</text>')
        parts.append(f'<text x="{pad - 8}" y="{sy(float(yv)):.1f}" font-family="sans-serif" font-size="11" text-anchor="end">{yv:.0f}</text>')
    parts.append("</svg>")
    path.write_text("\n".join(parts), encoding="utf-8")


def print_summary(result: FiveImuResult, out_csv: Path, out_svg: Path, out_json: Path) -> None:
    s = result.summary
    print(f"algorithm: {s['algorithm']}")
    print(f"samples: {s['sample_count']}  duration: {s['duration_s']:.1f} s  rate: {s['sample_rate_hz_median']:.1f} Hz")
    print("layout:", ", ".join(f"{role}={imu}" for role, imu in s["layout"].items()))
    for name, rel in s["relations"].items():
        drift = rel.get("static_drift_deg_per_min")
        drift_txt = f"{drift:.2f} deg/min" if drift is not None else "n/a"
        print(
            f"{name}: twist {rel['twist_min_deg']:.2f}..{rel['twist_max_deg']:.2f} deg, "
            f"RTZ {rel['return_to_zero_estimate_deg']:.2f} deg, drift {drift_txt}, "
            f"swing max {rel['swing_max_deg']:.2f} deg"
        )
    if "sternum_cross_check" in s:
        check = s["sternum_cross_check"]
        print(
            f"sternum cross-check ({check['anterior_relation']} vs {check['posterior_relation']}): "
            f"mean {check['angle_delta_mean_deg']:.2f} deg, max {check['angle_delta_max_deg']:.2f} deg"
        )
    if "cross_check_note" in s:
        print("cross-check note:", s["cross_check_note"])
    for warning in s.get("warnings", []):
        print(f"WARNING: {warning}")
    print(f"wrote {out_csv}\nwrote {out_svg}\nwrote {out_json}")


def make_demo_streams(layout: dict[str, str], rate_hz: float = 120.0, duration_s: float = 42.0) -> tuple[np.ndarray, dict[str, ImuStream]]:
    rng = np.random.default_rng(23)
    t = np.arange(int(duration_s * rate_hz)) / rate_hz
    role_quats = {role: np.zeros((len(t), 4), dtype=float) for role in layout}
    # Identify the chain order (root -> ... -> top/anterior) for whichever preset's roles are present.
    try:
        role_order = next(order for order in ROLE_ORDERS.values() if set(order) == set(layout))
    except StopIteration:
        raise SystemExit(f"No ROLE_ORDERS preset matches demo layout roles: {sorted(layout)}")
    # One mounting orientation per chain slot (slot 0 = root, slot 4 = top/anterior reference).
    mount_specs = [
        (np.array([0.0, 1.0, 0.0]), -90.0),
        (np.array([0.0, 1.0, 0.0]), 90.0),
        (np.array([1.0, 0.0, 0.0]), 90.0),
        (np.array([1.0, 0.0, 0.0]), -90.0),
        (np.array([0.0, 0.0, 1.0]), 0.0),
    ]
    mounts = [pair_fusion.quat_from_axis_angle(axis, math.radians(deg)) for axis, deg in mount_specs]
    for i, ti in enumerate(t):
        lumbar = pair_fusion.quat_from_axis_angle(np.array([0.0, 0.0, 1.0]), math.radians(18.0 * profile(ti, 8, 18)))
        thoracic = pair_fusion.quat_from_axis_angle(np.array([0.0, 0.0, 1.0]), math.radians(-12.0 * profile(ti, 20, 30)))
        bend = pair_fusion.quat_from_axis_angle(np.array([1.0, 0.0, 0.0]), math.radians(10.0 * profile(ti, 12, 24)))
        lumbar_bend = pair_fusion.qmul(lumbar, bend)
        lumbar_bend_thoracic = pair_fusion.qmul(lumbar_bend, thoracic)
        # Accumulated segment rotation per chain slot; slot 0 is the fixed root (no accumulation).
        accum = [None, lumbar, lumbar_bend, lumbar_bend_thoracic, lumbar_bend_thoracic]
        for slot, role in enumerate(role_order):
            if accum[slot] is None:
                role_quats[role][i] = mounts[slot]
            else:
                role_quats[role][i] = pair_fusion.qmul(accum[slot], mounts[slot])

    streams: dict[str, ImuStream] = {}
    for role, imu in layout.items():
        quats = role_quats[role]
        acc = acc_from_quats(quats) + rng.normal(0, 2.0, (len(t), 3))
        gyr = gyro_from_quats(quats, rate_hz) + rng.normal(0, 0.04, (len(t), 3)) + rng.normal(0, 0.2, 3)
        streams[imu.upper()] = ImuStream(imu=imu.upper(), acc_mg=acc, gyr_dps=gyr, q_sflp=pair_fusion.qnormalize(quats))
    return t, streams


def profile(t: float, start: float, end: float) -> float:
    up = smoothstep(t, start, start + 4.0)
    down = smoothstep(t, end - 4.0, end)
    return up - down


def smoothstep(t: float, start: float, end: float) -> float:
    if t <= start:
        return 0.0
    if t >= end:
        return 1.0
    x = (t - start) / (end - start)
    return x * x * (3.0 - 2.0 * x)


def acc_from_quats(quats: np.ndarray) -> np.ndarray:
    ez = np.array([0.0, 0.0, 1.0])
    return np.array([pair_fusion.quat_rotate(pair_fusion.qconj(q), ez) * 1000.0 for q in quats])


def gyro_from_quats(quats: np.ndarray, rate_hz: float) -> np.ndarray:
    gyr = np.zeros((len(quats), 3))
    for i in range(1, len(quats)):
        dq = pair_fusion.qmul(pair_fusion.qconj(quats[i - 1]), quats[i])
        if dq[0] < 0:
            dq = -dq
        angle = 2.0 * math.atan2(np.linalg.norm(dq[1:]), dq[0])
        axis = dq[1:] / max(1e-12, np.linalg.norm(dq[1:]))
        gyr[i] = np.degrees(axis * angle * rate_hz)
    return gyr


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="SpineSense five-IMU v1 relative orientation fusion.")
    parser.add_argument("--input", type=Path, help="5-IMU serial log.")
    parser.add_argument("--demo", action="store_true", help="Run a synthetic five-IMU chain check.")
    parser.add_argument("--filter", choices=("vqf", "madgwick", "sflp"), default="vqf")
    parser.add_argument("--auto-markers", action="store_true", help="Use matching *_markers.json for neutral windows.")
    parser.add_argument("--layout-preset", choices=tuple(LAYOUT_PRESETS), default="body5")
    parser.add_argument("--trial-id", help="Capture/trial id carried into placement provenance.")
    parser.add_argument("--placement-map-source", help="Authoritative registry entry or field-record source.")
    parser.add_argument("--mapping-version", help="Placement registry version.")
    parser.add_argument("--mapping-status", help="Placement review status (production: confirmed/inferred_high).")
    parser.add_argument("--mapping-sha256", help="Canonical SHA256 of the resolved placement entry.")
    parser.add_argument(
        "--allow-legacy-preset",
        action="store_true",
        help="Allow the quarantined, anatomically wrong t01 preset only to reproduce labelled legacy artifacts.",
    )
    for role in sorted({role for layout in LAYOUT_PRESETS.values() for role in layout}):
        parser.add_argument(f"--{role}", help=f"Override IMU id for {role}.")
    parser.add_argument("--up-axis", choices=("auto", *pair_fusion.NAMED_UP), default="auto")
    for role in sorted({role for layout in LAYOUT_PRESETS.values() for role in layout}):
        parser.add_argument(f"--{role}-up", choices=("auto", *pair_fusion.NAMED_UP), help=f"Override up axis for {role}.")
    parser.add_argument("--bias-seconds", type=float, default=8.0)
    parser.add_argument("--bias-start-s", type=float)
    parser.add_argument("--bias-end-s", type=float)
    parser.add_argument("--tare-seconds", type=float, default=8.0)
    parser.add_argument("--tare-start-s", type=float)
    parser.add_argument("--tare-end-s", type=float)
    parser.add_argument("--return-window-seconds", type=float, default=4.0)
    parser.add_argument("--end-still-start-s", type=float)
    parser.add_argument("--end-still-end-s", type=float)
    parser.add_argument("--drift-start-s", type=float, help="Drift window start (deg/min slope). Defaults to the end-still window.")
    parser.add_argument("--drift-end-s", type=float, help="Drift window end (deg/min slope). Defaults to the end-still window.")
    parser.add_argument("--still-gyro-std-max", type=float, default=0.5)
    parser.add_argument("--twist-axis-mode", choices=("fixed", "functional"), default="fixed",
                        help="Twist decomposition axis. 'fixed' = [0,0,1] (original). 'functional' estimates a "
                             "bend-decoupled longitudinal axis per relation from --twist-calib-window, cutting "
                             "flexion->twist cross-talk.")
    parser.add_argument("--twist-calib-window", type=float, nargs=2, action="append", metavar=("START", "END"),
                        help="Calibration window (trial seconds) for --twist-axis-mode functional; repeat for "
                             "several. Use pure-bend movements with NO axial twist: flexion AND lateral (or a "
                             "trunk circumduction). Flexion alone under-determines the axis and attenuates real "
                             "twist; pair it with lateral to pin the longitudinal axis near-vertical.")
    parser.add_argument("--out-csv", type=Path)
    parser.add_argument("--plot", type=Path)
    parser.add_argument("--summary", type=Path)
    return parser


def make_args(**overrides: object) -> argparse.Namespace:
    """Build an args namespace with CLI defaults, then apply overrides. Lets other tools (e.g.
    validation1_analysis) call run_pipeline without going through the command line. Mirrors
    twist_bench_fusion.make_args. Override keys use the argparse attribute form (underscores), e.g.
    layout_preset="validation1", end_still_start_s=28.0, t3="IMU2"."""
    args = build_parser().parse_args([])
    for key, value in overrides.items():
        if not hasattr(args, key):
            raise KeyError(f"unknown five_imu_fusion arg: {key!r}")
        setattr(args, key, value)
    return args


def main() -> int:
    args = build_parser().parse_args()
    result = run_pipeline(args.input, args)
    stem = "demo" if args.demo else args.input.stem
    out_csv, out_svg, out_json = write_outputs(result, stem, args)
    print_summary(result, out_csv, out_svg, out_json)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
