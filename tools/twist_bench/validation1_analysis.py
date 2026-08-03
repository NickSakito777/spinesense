from __future__ import annotations

"""SpineSense Validation1 analysis scaffold.

This is an ORCHESTRATION layer on top of five_imu_fusion.py. Its job for the MoCap pilot is to:

  raw 5-IMU log + trial metadata
    -> resolve sid->anatomical-role placement map (metadata OVERRIDES the preset default)
    -> parse ONCE, run the validation1 relative-orientation pipeline on a single common frame set
    -> slice that one result into ablation feature sets (no recapture)
    -> emit QC json + feature csv + human summary md

It deliberately does NOT decide a final classifier, fixed thresholds, sensor weights, or fuse/average
quaternions. It produces a clean, comparable feature table so a later step can draw conclusions with
MoCap ground truth in hand.

Orientation source: VQF (offline, raw accel/gyro) is the primary baseline. SFLP
(on-chip game-rotation-vector quaternion) can be selected with --filter sflp when the log carries
qw/qx/qy/qz columns; it uses the same relation/tare/swing-twist path as VQF.
"""

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import five_imu_fusion as fiv


PRESET = "validation1"
ROLES = list(fiv.VALIDATION1_LAYOUT)  # ["t3","t6","t12","sacrum","sternum"]

# How each relation should be read. The posterior chain is the primary kinematic signal; the
# composite is a redundant cross-check (~= product of the chain) and must NEVER be pooled with the
# chain; the sternum relations are anterior cross-check / features, never a correction.
RELATION_KIND = {
    "sacrum_to_t12": "posterior_chain",
    "t12_to_t6": "posterior_chain",
    "t6_to_t3": "posterior_chain",
    "sacrum_to_t3": "composite_crosscheck",
    "sacrum_to_sternum": "sternum_feature",
    "t3_to_sternum": "sternum_crosscheck",
    "t6_to_sternum": "sternum_crosscheck",
}

# Ablation feature sets, keyed by the IMU roles they keep. Sliced from ONE Full-B capture, never
# re-captured. posterior_only and no_sternum are the same IMU set by definition (removing sternum
# from Full B == posterior only); they are deduped by frozenset so the report does not double-count.
ABLATION_SETS = {
    "full_B": {"t3", "t6", "t12", "sacrum", "sternum"},
    "posterior_only": {"t3", "t6", "t12", "sacrum"},
    "no_T6": {"t3", "t12", "sacrum", "sternum"},
    "no_sternum": {"t3", "t6", "t12", "sacrum"},
    "minimal": {"t3", "t12", "sacrum"},
    "sternum_pelvis": {"sternum", "sacrum"},
}

FEATURE_FIELDS = [
    "ablation_set",
    "relation",
    "kind",
    "parent_role",
    "child_role",
    "twist_range_deg",
    "twist_abs_peak_deg",
    "twist_abs_peak_time_s",
    "return_to_zero_estimate_deg",
    "end_still_twist_rms_deg",
    "swing_max_deg",
    "static_drift_deg_per_min",
]


# --------------------------------------------------------------------------------------------------
# placement map resolution: CLI role flags > trial metadata > preset default
# --------------------------------------------------------------------------------------------------

def _normalize_to_role_to_imu(raw: dict) -> dict[str, str]:
    """Accept either {IMU0: t3, ...} (sid->role, the natural field-record form) or {t3: IMU0, ...}
    (role->IMU) and return role->IMU. Raises SystemExit on an unrecognized shape."""
    keys = [str(k) for k in raw]
    if keys and all(k.upper().startswith("IMU") for k in keys):
        return {str(v).lower(): str(k).upper() for k, v in raw.items()}  # IMU->role  ->  role->IMU
    return {str(k).lower(): str(v).upper() for k, v in raw.items()}      # already role->IMU


def load_placement_from_metadata(input_path: Path, config_arg: Path | None) -> tuple[dict[str, str] | None, str | None]:
    """Look for a placement_map in, in order: an explicit --config json; a sibling <stem>_layout.json;
    a top-level 'placement_map' inside the existing <stem>_markers.json. Returns (role->IMU, source)
    or (None, None) if none is found. Read-only; never writes the GUI's markers file."""
    candidates: list[tuple[Path, str]] = []
    if config_arg is not None:
        candidates.append((config_arg, "config"))
    if input_path is not None:
        candidates.append((input_path.with_name(input_path.stem + "_layout.json"), "sibling_layout"))
        candidates.append((input_path.with_name(input_path.stem + "_markers.json"), "markers"))
    for path, label in candidates:
        if not path.exists():
            continue
        try:
            blob = json.loads(path.read_text(encoding="utf-8"))
        except Exception as exc:  # noqa: BLE001 - a malformed metadata file should be loud, not silent
            raise SystemExit(f"{label} file {path} is not valid JSON: {exc}")
        raw = blob.get("placement_map") if isinstance(blob, dict) else None
        if raw:
            return _normalize_to_role_to_imu(raw), f"{label}:{path.name}"
    return None, None


def resolve_placement_map(args: argparse.Namespace) -> tuple[dict[str, str], str]:
    """Resolve the role->IMU map with precedence CLI > metadata > preset default, then validate it.
    Fail-fast on an incomplete or non-unique map: nothing downstream can recover a wrong mapping from
    the raw stream alone. An explicit source (metadata and/or CLI) must be COMPLETE on its own; we do
    NOT backfill an explicit-but-partial map from the preset default, because a half-specified map is
    a red flag, not something to silently complete with assumed defaults."""
    explicit: dict[str, str] = {}
    source: str | None = None

    meta_map, meta_source = load_placement_from_metadata(args.input, args.config)
    if meta_map:
        explicit.update(meta_map)
        source = meta_source

    cli_overrides = {role: getattr(args, role).upper() for role in ROLES if getattr(args, role, None)}
    if cli_overrides:
        explicit.update(cli_overrides)
        source = "cli" if source is None else f"cli+{source}"

    if not explicit:
        layout, source = dict(fiv.VALIDATION1_LAYOUT), "preset_default"
    else:
        layout = explicit
        missing = [role for role in ROLES if role not in layout or not layout[role]]
        if missing:
            raise SystemExit(
                f"placement map from {source} is incomplete; missing roles {missing}. "
                f"Specify all of {ROLES} (sid->role) for a Validation1 trial; the preset default is "
                f"used only when no metadata/CLI map is given at all."
            )

    missing = [role for role in ROLES if role not in layout or not layout[role]]
    if missing:
        raise SystemExit(f"placement map is incomplete; missing roles: {missing}. Record a full sid->role map.")
    imus = [layout[role] for role in ROLES]
    if len(set(imus)) != len(imus):
        raise SystemExit(f"placement map maps two roles to the same IMU: {layout}")
    return {role: layout[role] for role in ROLES}, source


# --------------------------------------------------------------------------------------------------
# SFLP availability
# --------------------------------------------------------------------------------------------------

def detect_sflp(input_path: Path | None) -> bool:
    """True only if the log carries on-chip quaternion columns."""
    if input_path is None or not input_path.exists():
        return False
    try:
        for line in input_path.read_text(encoding="utf-8", errors="replace").splitlines():
            toks = [t.lower() for t in line.split()]
            if "qw" in toks and "qx" in toks:  # SFLP firmware header: '... gz_dps qw qx qy qz'
                return True
    except Exception:  # noqa: BLE001
        return False
    return False


# --------------------------------------------------------------------------------------------------
# pipeline + ablation
# --------------------------------------------------------------------------------------------------

def build_fusion_args(args: argparse.Namespace, layout: dict[str, str]) -> argparse.Namespace:
    overrides: dict[str, object] = {
        "layout_preset": PRESET,
        "filter": args.filter,
        "demo": args.demo,
        "auto_markers": args.auto_markers,
    }
    overrides.update(layout)  # role->IMU, applied as --t3 IMUx etc.
    for win in ("bias_start_s", "bias_end_s", "tare_start_s", "tare_end_s", "end_still_start_s", "end_still_end_s"):
        value = getattr(args, win, None)
        if value is not None:
            overrides[win] = value
    return fiv.make_args(**overrides)


def feature_row(ablation_set: str, name: str, summary: dict) -> dict[str, object]:
    def g(key: str) -> object:
        v = summary.get(key)
        return round(float(v), 4) if isinstance(v, (int, float)) else v

    return {
        "ablation_set": ablation_set,
        "relation": name,
        "kind": RELATION_KIND.get(name, "other"),
        "parent_role": summary.get("parent_role"),
        "child_role": summary.get("child_role"),
        "twist_range_deg": g("twist_range_deg"),
        "twist_abs_peak_deg": g("twist_abs_peak_deg"),
        "twist_abs_peak_time_s": g("twist_abs_peak_time_s"),
        "return_to_zero_estimate_deg": g("return_to_zero_estimate_deg"),
        "end_still_twist_rms_deg": g("end_still_twist_rms_deg"),
        "swing_max_deg": g("swing_max_deg"),
        "static_drift_deg_per_min": g("static_drift_deg_per_min"),
    }


def build_ablation(result: fiv.FiveImuResult) -> tuple[list[dict[str, object]], dict[str, object]]:
    """Slice the single Full-B result into ablation sets. Dedupe identical IMU sets by frozenset so
    posterior_only/no_sternum collapse to one analysis (recorded as aliases)."""
    rows: list[dict[str, object]] = []
    sets_meta: dict[str, object] = {}
    seen: dict[frozenset, str] = {}

    for set_name, roles in ABLATION_SETS.items():
        key = frozenset(roles)
        if key in seen:
            primary = seen[key]
            sets_meta[primary].setdefault("aliases", []).append(set_name)
            continue
        seen[key] = set_name

        present, dropped = [], []
        for name, rel in result.relations.items():
            both_in = rel.summary["parent_role"] in roles and rel.summary["child_role"] in roles
            (present if both_in else dropped).append(name)
        for name in present:
            rows.append(feature_row(set_name, name, result.relations[name].summary))

        sets_meta[set_name] = {
            "roles": sorted(roles),
            "n_imus": len(roles),
            "available_relations": present,
            "dropped_relations": dropped,
            "aliases": [],
            "note": ("no adjacent posterior link available without the dropped segment(s)"
                     if not any(RELATION_KIND.get(n) == "posterior_chain" for n in present) else ""),
        }
    return rows, sets_meta


def build_qc(result: fiv.FiveImuResult, layout: dict[str, str], placement_source: str, sflp_available: bool,
             sets_meta: dict[str, object]) -> dict[str, object]:
    s = result.summary
    qc: dict[str, object] = {
        "algorithm": s["algorithm"],
        "input": s["input"],
        "layout_preset": PRESET,
        "placement_map": layout,
        "placement_map_source": placement_source,
        "axis_convention": "pending calibration",
        "axis_note": ("twist is decomposed about the fixed segment axis [0,0,1] after still-window "
                      "up-axis alignment; this equals the anatomical longitudinal axis only if the "
                      "alignment placed the true cranio-caudal axis on +z. Verify per landmark, esp. "
                      "the anterior sternum sensor."),
        "sflp_available": sflp_available,
        "orientation_source": s["algorithm"].split("_")[-1],
        "sample_count": s["sample_count"],
        "duration_s": s["duration_s"],
        "sample_rate_hz_median": s["sample_rate_hz_median"],
        "sample_dt_ms_p5_p95": s["sample_dt_ms_p5_p95"],
        "bias_window_s": s["bias_window_s"],
        "tare_window_s": s["tare_window_s"],
        "end_still_window_s": s.get("end_still_window_s"),
        "sensors": s["sensors"],
        "ablation_sets": sets_meta,
        "sternum_cross_check": s.get("sternum_cross_check"),
        "claim_boundary": s["claim_boundary"],
    }
    warnings = list(s.get("warnings", []))
    if placement_source == "preset_default":
        warnings.append(
            "placement_map is the UNVERIFIED preset default (sid->role assumed). Firmware sid is bound "
            "to a fixed socket, not anatomy; record the real sid->role map per trial for valid results."
        )
    if warnings:
        qc["warnings"] = warnings
    return qc


# --------------------------------------------------------------------------------------------------
# outputs
# --------------------------------------------------------------------------------------------------

def write_features_csv(rows: list[dict[str, object]], path: Path) -> None:
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=FEATURE_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: ("" if row.get(k) is None else row.get(k)) for k in FEATURE_FIELDS})


def write_summary_md(qc: dict[str, object], rows: list[dict[str, object]], path: Path) -> None:
    lines: list[str] = []
    lines.append("# SpineSense Validation1 feature + ablation summary")
    lines.append("")
    lines.append(f"- input: `{qc['input']}`")
    lines.append(f"- orientation source: `{qc['orientation_source']}` (VQF primary; SFLP cross-check); sflp_available: `{qc['sflp_available']}`")
    lines.append(f"- placement map source: `{qc['placement_map_source']}`  map: `{qc['placement_map']}`")
    lines.append(f"- samples: {qc['sample_count']}  duration: {qc['duration_s']:.1f} s  rate: {qc['sample_rate_hz_median']:.1f} Hz")
    lines.append(f"- axis convention: **{qc['axis_convention']}**")
    lines.append("")
    lines.append("> [!warning] What this scaffold does and does NOT claim")
    lines.append("> Outputs are short-term, neutral-tared, relative regional orientations from 6-axis")
    lines.append("> (magnetometer-free) IMUs: not absolute yaw, not 3D position, not per-vertebra rotation,")
    lines.append("> and **not a diagnosis** (PoC = posture monitoring + movement classification).")
    lines.append("> Axial twist is decomposed about a fixed segment axis pending anatomical-axis calibration.")
    lines.append("> VQF is the primary offline orientation source; SFLP is an on-chip cross-check when qw/qx/qy/qz are present.")
    lines.append("> Ablation comparability is defined on the single Full-B (all-5-present) frame set.")
    lines.append("")
    if qc.get("warnings"):
        lines.append("## Warnings")
        lines.append("")
        for w in qc["warnings"]:
            lines.append(f"- {w}")
        lines.append("")
    cc = qc.get("sternum_cross_check")
    if cc:
        lines.append("## Sternum cross-check (anterior vs posterior, a check not a correction)")
        lines.append("")
        lines.append(f"- {cc['anterior_relation']} vs {cc['posterior_relation']}: "
                     f"mean {cc['angle_delta_mean_deg']:.2f} deg, max {cc['angle_delta_max_deg']:.2f} deg")
        lines.append("")
    lines.append("## Ablation sets")
    lines.append("")
    lines.append("| set | IMUs | available relations | dropped | aliases |")
    lines.append("|---|---|---|---|---|")
    for name, meta in qc["ablation_sets"].items():
        lines.append(
            f"| {name} | {meta['n_imus']} ({', '.join(meta['roles'])}) | "
            f"{', '.join(meta['available_relations']) or '-'} | "
            f"{', '.join(meta['dropped_relations']) or '-'} | {', '.join(meta.get('aliases', [])) or '-'} |"
        )
    lines.append("")
    lines.append("## Features (per relation x ablation set)")
    lines.append("")
    header = ["set", "relation", "kind", "twist range", "abs peak", "peak t(s)", "RTZ", "swing max", "drift/min"]
    keys = ["ablation_set", "relation", "kind", "twist_range_deg", "twist_abs_peak_deg",
            "twist_abs_peak_time_s", "return_to_zero_estimate_deg", "swing_max_deg", "static_drift_deg_per_min"]
    lines.append("| " + " | ".join(header) + " |")
    lines.append("|" + "|".join(["---"] * len(header)) + "|")
    for row in rows:
        lines.append("| " + " | ".join("" if row.get(k) is None else str(row.get(k)) for k in keys) + " |")
    lines.append("")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


# --------------------------------------------------------------------------------------------------
# SFLP branch notes.
#
# Verified ISM6HG256X / LSM6DSV16X-family SFLP "game rotation vector" convention (from ST's own
# sflp2q() reference + AN6281 sec 8.6.3), to use when this branch is built:
#   * FIFO TAG_SENSOR == 0x13 carries ONLY the 3 vector components qx,qy,qz as IEEE binary16
#     half-floats, SCALAR-LAST. qw is NOT transmitted: reconstruct qw = +sqrt(max(0, 1 - sumsq)).
#   * The positive root loses qw sign and is unstable near 180 deg; ST clamps qw=0 when sumsq>1.
#   * VQF here is [w,x,y,z], sensor->world. Before any comparison: reorder SFLP to [w,x,y,z], run it
#     through the SAME quat_align_z_to up-axis alignment, and frame-check (conjugate if world->body).
#   * Compare with a DOUBLE-COVER-AWARE geodesic delta: canonicalize both quaternions to qw>=0, then
#     theta = 2*acos(|dot(q_sflp, q_vqf)|). NEVER compare components elementwise. Annotate/exclude
#     frames where SFLP sumsq is near 1 (qw near 0). Compare relative/twist + return-to-neutral only,
#     never absolute heading (both 6-DoF yaw drifts). Add a known +yaw/+pitch/+roll calibration spin
#     to lock sign+order before trusting any delta. Detect availability by FIFO tag 0x13 presence.
# The implementation now lives in five_imu_fusion.run_sensor_filter(filter="sflp").
# --------------------------------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="SpineSense Validation1 analysis scaffold (IMU-side, VQF primary).")
    parser.add_argument("--input", type=Path, help="5-IMU serial log.")
    parser.add_argument("--demo", action="store_true", help="Run on synthetic data (no hardware, preset-default map).")
    parser.add_argument("--config", type=Path, help="JSON with a top-level placement_map (sid->role).")
    parser.add_argument("--filter", choices=("vqf", "madgwick", "sflp"), default="vqf")
    parser.add_argument("--auto-markers", action="store_true", help="Use matching *_markers.json for neutral windows.")
    for role in ROLES:
        parser.add_argument(f"--{role}", help=f"Override IMU id for {role} (highest precedence).")
    parser.add_argument("--bias-start-s", type=float)
    parser.add_argument("--bias-end-s", type=float)
    parser.add_argument("--tare-start-s", type=float)
    parser.add_argument("--tare-end-s", type=float)
    parser.add_argument("--end-still-start-s", type=float)
    parser.add_argument("--end-still-end-s", type=float)
    parser.add_argument("--out-dir", type=Path, help="Output dir (default tools/twist_bench/plots).")
    return parser


def newest_log() -> Path:
    """Newest data/*.log next to this script (VSCode Run-with-no-args analyzes your latest capture)."""
    data_dir = Path(__file__).resolve().parent / "data"
    logs = sorted(data_dir.glob("*.log"), key=lambda p: p.stat().st_mtime, reverse=True)
    if not logs:
        raise SystemExit(f"No --input given and no .log files in {data_dir}")
    return logs[0]


def main() -> int:
    args = build_parser().parse_args()
    if not args.demo and args.input is None:
        args.input = newest_log()
        print(f"(no --input given; using newest log: {args.input.name})")
    if not args.demo and args.config is None:
        default_cfg = Path(__file__).resolve().parent / "data" / "validation1_layout.json"
        if default_cfg.exists():
            args.config = default_cfg
            print(f"(no --config given; using {default_cfg.name})")

    if args.demo:
        layout, placement_source = dict(fiv.VALIDATION1_LAYOUT), "preset_default"
    else:
        layout, placement_source = resolve_placement_map(args)
    sflp_available = detect_sflp(args.input)

    fusion_args = build_fusion_args(args, layout)
    result = fiv.run_pipeline(args.input, fusion_args)

    rows, sets_meta = build_ablation(result)
    qc = build_qc(result, layout, placement_source, sflp_available, sets_meta)

    stem = "demo" if args.demo else args.input.stem
    out_dir = args.out_dir or (Path(__file__).resolve().parent / "plots")
    out_dir.mkdir(parents=True, exist_ok=True)
    out_csv = out_dir / f"{stem}_validation1_features.csv"
    out_qc = out_dir / f"{stem}_validation1_qc.json"
    out_md = out_dir / f"{stem}_validation1_summary.md"

    write_features_csv(rows, out_csv)
    with out_qc.open("w", encoding="utf-8") as fh:
        json.dump(qc, fh, indent=2)
        fh.write("\n")
    write_summary_md(qc, rows, out_md)

    print(f"validation1 scaffold: {len(result.relations)} relations, "
          f"{len([k for k in sets_meta])} distinct ablation sets, sflp_available={sflp_available}")
    print(f"placement map ({placement_source}): " + ", ".join(f"{r}={layout[r]}" for r in ROLES))
    for w in qc.get("warnings", []):
        print(f"WARNING: {w}")
    print(f"wrote {out_csv}\nwrote {out_qc}\nwrote {out_md}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
