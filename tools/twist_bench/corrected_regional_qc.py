"""Placement-aware regional quaternion QC for the corrected 13-subject cohort.

This is a shadow-only audit runner.  It deliberately reuses
``dataset_adapter.subject_blocks`` as the frozen authority for raw
segments, clocks, protocol windows and detected bouts.  The only thing this
runner adds is a corrected anatomical regional readout and graph-consistency
QC; it does not repair or re-detect any bout.

The posterior spinal regions are exactly ``sacrum_to_lower``,
``lower_to_mid`` and ``mid_to_upper``.  ``upper_to_sternum`` is emitted only as
an anterior/posterior thorax cross-check, never as a fourth spinal region.
Whole-back and whole-trunk values come from the direct
``sacrum_to_upper`` and ``sacrum_to_sternum`` relations.

Quaternion chain closure is evaluated on raw ``q_rel`` values:

    q_SL * q_LM * q_MU == q_SU
    q_SL * q_LM * q_MU * q_US == q_S_sternum

Independently tared swing/twist scalars are never summed.  Closure is an
implementation/order check only; because all relations share the same node
orientations, it is not independent evidence that a placement map is correct.

``--out-dir`` is mandatory, must be new or empty, and must not resolve inside
``data_clean``.  ``--dry-check`` validates the registry, manifests, raw paths
and output safety without importing the numerical pipeline or loading trials.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import platform
from pathlib import Path
import sys
from typing import Any, Iterable

import numpy as np


HERE = Path(__file__).resolve().parent
VAULT = HERE.parents[2]
DATA_CLEAN = HERE / "data_clean"
DEFAULT_REGISTRY = HERE / "config" / "placement_maps_v1.json"
# Cohort comes from the dataset config, not from a constant here -- a hard-coded
# session list makes the script describe one study rather than one method.
sys.path.insert(0, str(HERE))
import placement_maps as pm  # noqa: E402 -- safe for dry-check
from coverage_gate import assess_scoring_coverage  # noqa: E402


SCHEMA_VERSION = 1
CLOSURE_TOLERANCE_DEG = 1e-4
MIN_TARE_SAMPLES = 3
MIN_BOUT_SAMPLES = 5
RETURN_START_OFFSET_S = 0.2
RETURN_END_OFFSET_S = 1.2

RELATION_SPECS: tuple[dict[str, Any], ...] = (
    {
        "relation": "sacrum_to_lower",
        "parent_role": "sacrum",
        "child_role": "lower",
        "semantic_type": "posterior_region",
        "posterior_order": 1,
        "is_spinal_region": True,
    },
    {
        "relation": "lower_to_mid",
        "parent_role": "lower",
        "child_role": "mid",
        "semantic_type": "posterior_region",
        "posterior_order": 2,
        "is_spinal_region": True,
    },
    {
        "relation": "mid_to_upper",
        "parent_role": "mid",
        "child_role": "upper",
        "semantic_type": "posterior_region",
        "posterior_order": 3,
        "is_spinal_region": True,
    },
    {
        "relation": "upper_to_sternum",
        "parent_role": "upper",
        "child_role": "sternum",
        "semantic_type": "thorax_crosscheck",
        "posterior_order": None,
        "is_spinal_region": False,
    },
    {
        "relation": "sacrum_to_upper",
        "parent_role": "sacrum",
        "child_role": "upper",
        "semantic_type": "whole_back_direct",
        "posterior_order": None,
        "is_spinal_region": False,
    },
    {
        "relation": "sacrum_to_sternum",
        "parent_role": "sacrum",
        "child_role": "sternum",
        "semantic_type": "whole_trunk_direct",
        "posterior_order": None,
        "is_spinal_region": False,
    },
)

RELATION_BY_NAME = {spec["relation"]: spec for spec in RELATION_SPECS}

METRIC_FIELDS = (
    "subject",
    "trial_id",
    "block",
    "label",
    "quality",
    "bout_index",
    "bout_start_mocap_s",
    "bout_end_mocap_s",
    "clock_a",
    "clock_b",
    "mapping_version",
    "mapping_sha256",
    "relation",
    "semantic_type",
    "posterior_order",
    "is_spinal_region",
    "parent_role",
    "child_role",
    "parent_imu",
    "child_imu",
    "status",
    "n_window_samples",
    "n_valid_samples",
    "valid_fraction",
    "n_tare_samples",
    "n_return_samples",
    "swing_rom_deg",
    "twist_rom_deg",
    "geodesic_rom_deg",
    "swing_peak_deg",
    "twist_abs_peak_deg",
    "return_to_zero_rotation_mean_deg",
    "return_to_zero_rotation_last_deg",
    "return_to_zero_swing_mean_deg",
    "return_to_zero_twist_abs_mean_deg",
    "return_to_zero_twist_signed_mean_deg",
)

CLOSURE_FIELDS = (
    "subject",
    "trial_id",
    "block",
    "label",
    "quality",
    "bout_index",
    "bout_start_mocap_s",
    "bout_end_mocap_s",
    "clock_a",
    "clock_b",
    "mapping_version",
    "mapping_sha256",
    "closure_id",
    "raw_relation_path",
    "direct_relation",
    "status",
    "n_window_samples",
    "n_valid_samples",
    "valid_fraction",
    "closure_mean_deg",
    "closure_rms_deg",
    "closure_p95_deg",
    "closure_max_deg",
    "tolerance_deg",
    "algorithmic_closure_pass",
    "interpretation",
)

_FILE_RECORD_CACHE: dict[Path, dict[str, Any]] = {}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--out-dir",
        type=Path,
        required=True,
        help="New/empty shadow output directory; paths inside data_clean are rejected.",
    )
    parser.add_argument(
        "--registry",
        type=Path,
        default=DEFAULT_REGISTRY,
        help="Corrected per-trial placement registry.",
    )
    parser.add_argument(
        "--subjects",
        nargs="+",
        default=list(mlb.sessions()),
        help="Subject ids such as 12 or T12 (default: all 13 usable subjects).",
    )
    parser.add_argument(
        "--quality-manifest",
        type=Path,
        required=True,
        help="Final corrected_uniform block_quality_v1.json used for quality and coverage provenance.",
    )
    parser.add_argument(
        "--dry-check",
        action="store_true",
        help="Validate paths/registry/manifests/output safety; load no trial data and write nothing.",
    )
    return parser


def normalize_subjects(values: Iterable[str]) -> list[str]:
    subjects: list[str] = []
    for value in values:
        subject = str(value).strip().upper()
        if subject.startswith("T"):
            subject = subject[1:]
        if not subject.isdigit():
            raise SystemExit(f"invalid subject id {value!r}")
        subject = f"{int(subject):02d}"
        if subject not in mlb.sessions():
            raise SystemExit(
                f"subject T{subject} is outside the 13-subject validation cohort: "
                f"{', '.join('T' + item for item in mlb.sessions())}"
            )
        if subject not in subjects:
            subjects.append(subject)
    if not subjects:
        raise SystemExit("--subjects resolved to an empty cohort")
    return subjects


def trial_id(subject: str) -> str:
    return f"T{subject}_P{subject}"


def manifest_path(subject: str) -> Path:
    return DATA_CLEAN / trial_id(subject) / "block_manifest.json"


def mocap_path(subject: str) -> Path:
    return HERE / "data" / "mocap" / f"P{subject}_all.csv"


def _is_within(path: Path, parent: Path) -> bool:
    try:
        path.relative_to(parent)
        return True
    except ValueError:
        return False


def validate_out_dir(path: Path, *, dry_check: bool) -> Path:
    out_dir = path.expanduser().resolve()
    if _is_within(out_dir, DATA_CLEAN.resolve()):
        raise SystemExit(f"refusing shadow output inside data_clean: {out_dir}")
    if out_dir.exists() and not out_dir.is_dir():
        raise SystemExit(f"--out-dir exists but is not a directory: {out_dir}")
    if out_dir.exists() and any(out_dir.iterdir()):
        raise SystemExit(f"--out-dir must be new or empty; refusing to overwrite: {out_dir}")
    if not dry_check:
        out_dir.mkdir(parents=True, exist_ok=True)
    return out_dir


def _manifest_blocks(manifest: dict[str, Any]) -> list[tuple[str, dict[str, Any]]]:
    blocks = manifest.get("blocks")
    if isinstance(blocks, list):
        result: list[tuple[str, dict[str, Any]]] = []
        for index, block in enumerate(blocks):
            if not isinstance(block, dict):
                raise ValueError(f"blocks[{index}] is not an object")
            block_id = block.get("id") or block.get("block") or block.get("block_id")
            if not block_id:
                raise ValueError(f"blocks[{index}] has no id")
            result.append((str(block_id), block))
        return result
    if isinstance(blocks, dict):
        if not all(isinstance(block, dict) for block in blocks.values()):
            raise ValueError("one or more mapping-schema blocks are not objects")
        return [(str(block_id), block) for block_id, block in blocks.items()]
    raise ValueError("manifest.blocks must be a list or object")


def _resolve_registry_raw(raw: str | Path) -> Path:
    path = Path(raw)
    if path.is_absolute():
        return path
    return HERE / path


def _resolve_source_ref(source_ref: str | Path) -> Path | None:
    path = Path(source_ref)
    candidates = [path] if path.is_absolute() else [VAULT / path, HERE / path]
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    return None


def load_quality_manifest(
    path: Path, registry_path: Path
) -> tuple[dict[tuple[str, str], dict[str, Any]], dict[str, Any]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("run_type") != "corrected_uniform_block_quality":
        raise ValueError("quality manifest is not corrected_uniform_block_quality")
    if payload.get("mapping_registry_sha256") != sha256_file(registry_path):
        raise ValueError("quality manifest registry hash mismatch")
    rows: dict[tuple[str, str], dict[str, Any]] = {}
    for row in payload.get("rows", []):
        key = (str(row.get("subject")), str(row.get("block")))
        if key in rows:
            raise ValueError(f"duplicate quality row {key}")
        rows[key] = row
    return rows, payload


def dry_check(
    subjects: list[str], registry_path: Path, quality_manifest: Path, out_dir: Path
) -> int:
    registry_path = registry_path.expanduser().resolve()
    registry = pm.load_placement_registry(registry_path)
    errors: list[str] = []
    try:
        quality_rows, _ = load_quality_manifest(quality_manifest.resolve(), registry_path)
    except Exception as exc:
        quality_rows = {}
        errors.append(f"invalid quality manifest: {exc}")
    for subject in subjects:
        tid = trial_id(subject)
        try:
            placement = pm.resolve_placement(trial_id=tid, config_path=registry_path)
        except Exception as exc:
            errors.append(f"{tid}: placement: {exc}")
            continue
        if placement.trial_id not in registry:
            errors.append(f"{tid}: placement vanished after eager registry validation")
        expected_blocks: list[str] = []
        man_path = manifest_path(subject)
        if not man_path.is_file():
            errors.append(f"{tid}: missing frozen block manifest {man_path}")
        else:
            try:
                manifest = json.loads(man_path.read_text(encoding="utf-8"))
                manifest_blocks = _manifest_blocks(manifest)
                expected_blocks = [block_id for block_id, _ in manifest_blocks]
                if not manifest_blocks:
                    errors.append(f"{tid}: manifest contains no blocks")
            except Exception as exc:
                errors.append(f"{tid}: invalid manifest: {exc}")
        moc_path = mocap_path(subject)
        if not moc_path.is_file():
            errors.append(f"{tid}: missing MoCap {moc_path}")
        missing_raw = [
            str(_resolve_registry_raw(raw))
            for raw in placement.raw_segments
            if not _resolve_registry_raw(raw).is_file()
        ]
        if missing_raw:
            errors.append(f"{tid}: registry raw paths missing: {missing_raw}")
        for block_id in expected_blocks:
            if (f"T{subject}", block_id) not in quality_rows:
                errors.append(f"{tid}: quality manifest missing block {block_id}")

    dependencies = (
        HERE / "dataset_adapter.py",
        HERE / "five_imu_fusion.py",
        HERE / "twist_bench_fusion.py",
        HERE / "session_recipe.py",
        HERE / "session_recipe.py",
        HERE / "signed_diagnostic.py",
    )
    missing_dependencies = [str(path) for path in dependencies if not path.is_file()]
    if missing_dependencies:
        errors.append(f"missing numerical dependencies: {missing_dependencies}")
    if errors:
        raise SystemExit("dry-check failed:\n- " + "\n- ".join(errors))
    print(
        f"dry-check OK: {len(subjects)} subject(s), registry={registry_path}, "
        f"shadow_out={out_dir} (not created)"
    )
    return 0


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def file_record(path: Path) -> dict[str, Any]:
    resolved = path.expanduser().resolve()
    cached = _FILE_RECORD_CACHE.get(resolved)
    if cached is not None:
        return dict(cached)
    record = {
        "path": str(resolved),
        "size_bytes": int(resolved.stat().st_size),
        "sha256": sha256_file(resolved),
    }
    _FILE_RECORD_CACHE[resolved] = record
    return dict(record)


def _jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, (np.integer, np.floating)):
        return value.item()
    return value


def _finite(value: float) -> float | None:
    return round(float(value), 9) if math.isfinite(float(value)) else None


def _valid_quat_rows(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    if q.ndim != 2 or q.shape[1] != 4:
        return np.zeros(len(q) if q.ndim else 0, dtype=bool)
    return np.all(np.isfinite(q), axis=1) & (np.linalg.norm(q, axis=1) > 1e-9)


def _quat_geodesic_deg(q: np.ndarray, pf: Any) -> np.ndarray:
    q = pf.qnormalize(np.asarray(q, dtype=float))
    return np.degrees(2.0 * np.arccos(np.clip(np.abs(q[:, 0]), 0.0, 1.0)))


def validate_result_mapping(res: Any, placement: pm.ResolvedPlacement) -> None:
    layout = dict(res.summary.get("layout", {}))
    expected_layout = dict(placement.role_to_imu)
    if layout != expected_layout:
        raise RuntimeError(
            f"{placement.trial_id}: fusion layout does not match registry: "
            f"result={layout}, registry={expected_layout}"
        )
    provenance = res.summary.get("placement_provenance", {})
    required_provenance = {
        "trial_id": placement.trial_id,
        "mapping_version": placement.mapping_version,
        "mapping_status": placement.status,
        "mapping_sha256": placement.canonical_sha256,
    }
    mismatches = {
        key: {"result": provenance.get(key), "registry": value}
        for key, value in required_provenance.items()
        if provenance.get(key) != value
    }
    if mismatches:
        raise RuntimeError(f"{placement.trial_id}: fusion placement provenance mismatch: {mismatches}")

    for spec in RELATION_SPECS:
        name = spec["relation"]
        if name not in res.relations:
            raise RuntimeError(f"{placement.trial_id}: fusion result lacks relation {name}")
        relation = res.relations[name]
        expected = (
            spec["parent_role"],
            spec["child_role"],
            expected_layout[spec["parent_role"]],
            expected_layout[spec["child_role"]],
        )
        actual = (
            relation.parent_role,
            relation.child_role,
            relation.summary.get("parent_imu"),
            relation.summary.get("child_imu"),
        )
        if actual != expected:
            raise RuntimeError(
                f"{placement.trial_id}: relation endpoint mismatch for {name}: "
                f"result={actual}, registry={expected}"
            )


def relation_bout_metrics(
    *,
    res: Any,
    spec: dict[str, Any],
    placement: pm.ResolvedPlacement,
    subject: str,
    block_id: str,
    label: str,
    quality: str,
    bout_index: int,
    lo: float,
    hi: float,
    a: float,
    b: float,
    pf: Any,
    t06: Any,
) -> dict[str, Any]:
    relation = res.relations[spec["relation"]]
    q = np.asarray(relation.q_rel, dtype=float)
    ti = np.asarray(res.t_s, dtype=float)
    if len(q) != len(ti):
        raise RuntimeError(
            f"{placement.trial_id}/{spec['relation']}: q_rel length {len(q)} != time length {len(ti)}"
        )

    window = (ti >= a * lo + b) & (ti <= a * hi + b)
    valid_q = _valid_quat_rows(q)
    tare = np.asarray(t06.pre_mask(ti, a, b, lo), dtype=bool)
    tare_valid = tare & valid_q
    # peak_bouts ends at the detector's 35%-of-peak crossing, not at neutral.
    # Measure return after that boundary, mirroring the pre-bout neutral window.
    return_mask = (
        (ti >= a * (hi + RETURN_START_OFFSET_S) + b)
        & (ti <= a * (hi + RETURN_END_OFFSET_S) + b)
    )

    n_window = int(np.count_nonzero(window))
    n_valid = int(np.count_nonzero(window & valid_q))
    n_tare = int(np.count_nonzero(tare_valid))
    base = {
        "subject": f"T{subject}",
        "trial_id": placement.trial_id,
        "block": block_id,
        "label": label,
        "quality": quality,
        "bout_index": bout_index,
        "bout_start_mocap_s": round(float(lo), 6),
        "bout_end_mocap_s": round(float(hi), 6),
        "clock_a": float(a),
        "clock_b": float(b),
        "mapping_version": placement.mapping_version,
        "mapping_sha256": placement.canonical_sha256,
        "relation": spec["relation"],
        "semantic_type": spec["semantic_type"],
        "posterior_order": spec["posterior_order"],
        "is_spinal_region": bool(spec["is_spinal_region"]),
        "parent_role": spec["parent_role"],
        "child_role": spec["child_role"],
        "parent_imu": placement.role_to_imu[spec["parent_role"]],
        "child_imu": placement.role_to_imu[spec["child_role"]],
        "status": "ok",
        "n_window_samples": n_window,
        "n_valid_samples": n_valid,
        "valid_fraction": round(n_valid / n_window, 9) if n_window else 0.0,
        "n_tare_samples": n_tare,
        "n_return_samples": 0,
        "swing_rom_deg": None,
        "twist_rom_deg": None,
        "geodesic_rom_deg": None,
        "swing_peak_deg": None,
        "twist_abs_peak_deg": None,
        "return_to_zero_rotation_mean_deg": None,
        "return_to_zero_rotation_last_deg": None,
        "return_to_zero_swing_mean_deg": None,
        "return_to_zero_twist_abs_mean_deg": None,
        "return_to_zero_twist_signed_mean_deg": None,
    }
    if n_window < MIN_BOUT_SAMPLES:
        base["status"] = "insufficient_window_samples"
        return base
    if n_tare < MIN_TARE_SAMPLES:
        base["status"] = "insufficient_tare_samples"
        return base
    if n_valid < MIN_BOUT_SAMPLES:
        base["status"] = "insufficient_valid_samples"
        return base

    q0 = pf.quat_average(q[tare_valid])
    q_tared = np.full_like(q, np.nan, dtype=float)
    q_tared[valid_q] = pf.qmul(pf.qconj(q0)[None, :], q[valid_q])
    twist_valid, swing_valid = pf.swing_twist_deg(
        q_tared[valid_q], pf.SEGMENT_TWIST_AXIS
    )
    twist_valid = pf.unwrap_deg(twist_valid)
    geodesic_valid = _quat_geodesic_deg(q_tared[valid_q], pf)

    twist = np.full(len(ti), np.nan, dtype=float)
    swing = np.full(len(ti), np.nan, dtype=float)
    geodesic = np.full(len(ti), np.nan, dtype=float)
    twist[valid_q] = twist_valid
    swing[valid_q] = swing_valid
    geodesic[valid_q] = geodesic_valid

    bout_valid = window & valid_q
    twist_bout = twist[bout_valid]
    swing_bout = swing[bout_valid]
    geodesic_bout = geodesic[bout_valid]
    base.update(
        {
            "swing_rom_deg": _finite(np.ptp(swing_bout)),
            "twist_rom_deg": _finite(np.ptp(twist_bout)),
            "geodesic_rom_deg": _finite(np.ptp(geodesic_bout)),
            "swing_peak_deg": _finite(np.max(swing_bout)),
            "twist_abs_peak_deg": _finite(np.max(np.abs(twist_bout))),
        }
    )

    return_valid = return_mask & valid_q
    n_return = int(np.count_nonzero(return_valid))
    base["n_return_samples"] = n_return
    if n_return:
        rotation_return = geodesic[return_valid]
        swing_return = swing[return_valid]
        twist_return = twist[return_valid]
        base.update(
            {
                "return_to_zero_rotation_mean_deg": _finite(np.mean(rotation_return)),
                "return_to_zero_rotation_last_deg": _finite(rotation_return[-1]),
                "return_to_zero_swing_mean_deg": _finite(np.mean(swing_return)),
                "return_to_zero_twist_abs_mean_deg": _finite(np.mean(np.abs(twist_return))),
                "return_to_zero_twist_signed_mean_deg": _finite(np.mean(twist_return)),
            }
        )
    return base


def build_closure_series(res: Any, pf: Any) -> dict[str, dict[str, Any]]:
    q = {
        name: np.asarray(res.relations[name].q_rel, dtype=float)
        for name in (
            "sacrum_to_lower",
            "lower_to_mid",
            "mid_to_upper",
            "upper_to_sternum",
            "sacrum_to_upper",
            "sacrum_to_sternum",
        )
    }
    lengths = {name: len(values) for name, values in q.items()}
    if len(set(lengths.values())) != 1 or next(iter(lengths.values())) != len(res.t_s):
        raise RuntimeError(f"relation length mismatch while building closure series: {lengths}")

    valid = {name: _valid_quat_rows(values) for name, values in q.items()}

    posterior_valid = (
        valid["sacrum_to_lower"]
        & valid["lower_to_mid"]
        & valid["mid_to_upper"]
        & valid["sacrum_to_upper"]
    )
    full_valid = (
        posterior_valid
        & valid["upper_to_sternum"]
        & valid["sacrum_to_sternum"]
    )

    def closure_error(chain: np.ndarray, direct: np.ndarray) -> np.ndarray:
        delta = pf.qmul(pf.qconj(chain), direct)
        return _quat_geodesic_deg(delta, pf)

    posterior_error = np.full(len(res.t_s), np.nan, dtype=float)
    chain_posterior = pf.qmul(
        pf.qmul(q["sacrum_to_lower"][posterior_valid], q["lower_to_mid"][posterior_valid]),
        q["mid_to_upper"][posterior_valid],
    )
    posterior_error[posterior_valid] = closure_error(
        chain_posterior, q["sacrum_to_upper"][posterior_valid]
    )

    full_error = np.full(len(res.t_s), np.nan, dtype=float)
    chain_posterior_full = pf.qmul(
        pf.qmul(q["sacrum_to_lower"][full_valid], q["lower_to_mid"][full_valid]),
        q["mid_to_upper"][full_valid],
    )
    chain_full = pf.qmul(chain_posterior_full, q["upper_to_sternum"][full_valid])
    full_error[full_valid] = closure_error(
        chain_full, q["sacrum_to_sternum"][full_valid]
    )
    return {
        "posterior_chain_to_direct_su": {
            "valid": posterior_valid,
            "error_deg": posterior_error,
            "raw_relation_path": "sacrum_to_lower*lower_to_mid*mid_to_upper",
            "direct_relation": "sacrum_to_upper",
        },
        "full_graph_to_direct_s_sternum": {
            "valid": full_valid,
            "error_deg": full_error,
            "raw_relation_path": "sacrum_to_lower*lower_to_mid*mid_to_upper*upper_to_sternum",
            "direct_relation": "sacrum_to_sternum",
        },
    }


def closure_bout_rows(
    *,
    res: Any,
    closure_series: dict[str, dict[str, Any]],
    placement: pm.ResolvedPlacement,
    subject: str,
    block_id: str,
    label: str,
    quality: str,
    bout_index: int,
    lo: float,
    hi: float,
    a: float,
    b: float,
) -> list[dict[str, Any]]:
    ti = np.asarray(res.t_s, dtype=float)
    window = (ti >= a * lo + b) & (ti <= a * hi + b)
    n_window = int(np.count_nonzero(window))
    rows: list[dict[str, Any]] = []
    for closure_id, series in closure_series.items():
        valid = window & series["valid"]
        values = series["error_deg"][valid]
        n_valid = int(len(values))
        status = "ok" if n_window >= MIN_BOUT_SAMPLES and n_valid >= MIN_BOUT_SAMPLES else "insufficient_valid_samples"
        maximum = float(np.max(values)) if n_valid else math.nan
        rows.append(
            {
                "subject": f"T{subject}",
                "trial_id": placement.trial_id,
                "block": block_id,
                "label": label,
                "quality": quality,
                "bout_index": bout_index,
                "bout_start_mocap_s": round(float(lo), 6),
                "bout_end_mocap_s": round(float(hi), 6),
                "clock_a": float(a),
                "clock_b": float(b),
                "mapping_version": placement.mapping_version,
                "mapping_sha256": placement.canonical_sha256,
                "closure_id": closure_id,
                "raw_relation_path": series["raw_relation_path"],
                "direct_relation": series["direct_relation"],
                "status": status,
                "n_window_samples": n_window,
                "n_valid_samples": n_valid,
                "valid_fraction": round(n_valid / n_window, 9) if n_window else 0.0,
                "closure_mean_deg": _finite(np.mean(values)) if n_valid else None,
                "closure_rms_deg": _finite(np.sqrt(np.mean(values * values))) if n_valid else None,
                "closure_p95_deg": _finite(np.percentile(values, 95)) if n_valid else None,
                "closure_max_deg": _finite(maximum) if n_valid else None,
                "tolerance_deg": CLOSURE_TOLERANCE_DEG,
                "algorithmic_closure_pass": bool(
                    status == "ok" and maximum <= CLOSURE_TOLERANCE_DEG
                ),
                "interpretation": "raw quaternion implementation/order check; not independent placement validation",
            }
        )
    return rows


def _write_csv(path: Path, fields: tuple[str, ...], rows: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(fields), extrasaction="raise")
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in fields})


def _median(values: Iterable[float | None]) -> float | None:
    finite = [float(value) for value in values if value is not None and math.isfinite(float(value))]
    return _finite(np.median(finite)) if finite else None


def summarize_relations(metric_rows: list[dict[str, Any]]) -> dict[str, Any]:
    summary: dict[str, Any] = {}
    for spec in RELATION_SPECS:
        rows = [row for row in metric_rows if row["relation"] == spec["relation"]]
        summary[spec["relation"]] = {
            "semantic_type": spec["semantic_type"],
            "n_rows": len(rows),
            "n_ok": sum(row["status"] == "ok" for row in rows),
            "median_valid_fraction": _median(row["valid_fraction"] for row in rows),
            "median_swing_rom_deg": _median(row["swing_rom_deg"] for row in rows),
            "median_twist_rom_deg": _median(row["twist_rom_deg"] for row in rows),
            "median_return_to_zero_rotation_mean_deg": _median(
                row["return_to_zero_rotation_mean_deg"] for row in rows
            ),
        }
    return summary


def summarize_closure(closure_rows: list[dict[str, Any]]) -> dict[str, Any]:
    summary: dict[str, Any] = {}
    for closure_id in (
        "posterior_chain_to_direct_su",
        "full_graph_to_direct_s_sternum",
    ):
        rows = [row for row in closure_rows if row["closure_id"] == closure_id]
        maxima = [
            float(row["closure_max_deg"])
            for row in rows
            if row["closure_max_deg"] is not None
        ]
        summary[closure_id] = {
            "n_rows": len(rows),
            "n_pass": sum(bool(row["algorithmic_closure_pass"]) for row in rows),
            "n_insufficient": sum(row["status"] != "ok" for row in rows),
            "max_closure_deg": _finite(max(maxima)) if maxima else None,
            "median_closure_max_deg": _median(maxima),
            "tolerance_deg": CLOSURE_TOLERANCE_DEG,
        }
    return summary


def run_regional_qc(
    subjects: list[str],
    registry_path: Path,
    quality_manifest_path: Path,
    out_dir: Path,
    command_args: list[str],
) -> int:
    # Delayed imports preserve a read-only, no-trial-data dry-check path.
    import dataset_adapter as mlb  # type: ignore  # noqa: PLC0415
    import five_imu_fusion as fiv  # noqa: PLC0415
    import session_recipe as t06  # noqa: PLC0415
    import signed_diagnostic as sd  # noqa: PLC0415
    import twist_bench_fusion as pf  # noqa: PLC0415

    registry_path = registry_path.expanduser().resolve()
    quality_manifest_path = quality_manifest_path.expanduser().resolve()
    quality_rows, quality_payload = load_quality_manifest(
        quality_manifest_path, registry_path
    )
    placements = {
        trial_id(subject): pm.resolve_placement(
            trial_id=trial_id(subject), config_path=registry_path
        )
        for subject in subjects
    }

    original_fusion_args = mlb.fusion_args

    def configured_fusion_args(subject: str, imu: Path, filter_name: str):
        placement = pm.resolve_placement(
            trial_id=trial_id(subject),
            raw_path=imu,
            config_path=registry_path,
        )
        return fiv.make_args(filter=filter_name, **placement.fusion_kwargs())

    mlb.fusion_args = configured_fusion_args
    metric_rows: list[dict[str, Any]] = []
    closure_rows: list[dict[str, Any]] = []
    subject_payloads: dict[str, dict[str, Any]] = {}
    all_used_raw_paths: set[Path] = set()
    subject_json_paths: list[Path] = []
    try:
        for subject in subjects:
            tid = trial_id(subject)
            placement = placements[tid]
            tm, flex, lat, axial = sd.mocap_signed(mocap_path(subject))
            signals = {"flex": flex, "lat": lat, "axial": axial}
            subject_metric_start = len(metric_rows)
            subject_closure_start = len(closure_rows)
            block_summaries: list[dict[str, Any]] = []
            seen_blocks: set[str] = set()
            validated_results: set[int] = set()
            closure_cache: dict[int, dict[str, dict[str, Any]]] = {}
            subject_used_raw_paths: set[Path] = set()

            for block_id, block, res, a, b, bouts in mlb.subject_blocks(subject, tm, signals):
                if block_id in seen_blocks:
                    raise RuntimeError(f"{tid}: frozen subject_blocks yielded duplicate block {block_id}")
                seen_blocks.add(block_id)
                override = mlb.BLOCK_OVERRIDES.get(subject, {}).get(block_id, {})
                label = mlb._canon_label(override.get("label", block["label"]))
                if label not in mlb.LABELS:
                    continue
                quality_row = quality_rows[(f"T{subject}", block_id)]
                if quality_row.get("mapping_sha256") != placement.canonical_sha256:
                    raise RuntimeError(f"{tid}/{block_id}: quality mapping hash mismatch")
                quality = str(quality_row["quality"])

                result_key = id(res)
                if result_key not in validated_results:
                    validate_result_mapping(res, placement)
                    validated_results.add(result_key)
                    closure_cache[result_key] = build_closure_series(res, pf)
                source_input = Path(str(res.summary.get("input", "")))
                if source_input.is_file():
                    source_input = source_input.resolve()
                    subject_used_raw_paths.add(source_input)
                    all_used_raw_paths.add(source_input)

                indexed_bouts = list(enumerate(bouts))
                _, coverage = assess_scoring_coverage(
                    res.t_s, float(a), float(b), bouts, np
                )
                indexed_bouts = [
                    item for item, qc in zip(indexed_bouts, coverage) if qc["accepted"]
                ]
                block_metric_start = len(metric_rows)
                block_closure_start = len(closure_rows)
                for bout_index, (lo_value, hi_value) in indexed_bouts:
                    lo, hi = float(lo_value), float(hi_value)
                    if not math.isfinite(lo) or not math.isfinite(hi) or hi <= lo:
                        raise RuntimeError(f"{tid}/{block_id}: invalid frozen bout {(lo, hi)}")
                    for spec in RELATION_SPECS:
                        metric_rows.append(
                            relation_bout_metrics(
                                res=res,
                                spec=spec,
                                placement=placement,
                                subject=subject,
                                block_id=block_id,
                                label=label,
                                quality=quality,
                                bout_index=bout_index,
                                lo=lo,
                                hi=hi,
                                a=float(a),
                                b=float(b),
                                pf=pf,
                                t06=t06,
                            )
                        )
                    closure_rows.extend(
                        closure_bout_rows(
                            res=res,
                            closure_series=closure_cache[result_key],
                            placement=placement,
                            subject=subject,
                            block_id=block_id,
                            label=label,
                            quality=quality,
                            bout_index=bout_index,
                            lo=lo,
                            hi=hi,
                            a=float(a),
                            b=float(b),
                        )
                    )

                block_metric_rows = metric_rows[block_metric_start:]
                block_closure_rows = closure_rows[block_closure_start:]
                block_summaries.append(
                    {
                        "block": block_id,
                        "label": label,
                        "quality": quality,
                        "clock": {"a": float(a), "b": float(b)},
                        "detected_bouts": [[float(lo), float(hi)] for lo, hi in bouts],
                        "scored_bouts": [[float(lo), float(hi)] for _, (lo, hi) in indexed_bouts],
                        "n_bouts_detected": len(bouts),
                        "n_bouts_scored": len(indexed_bouts),
                        "n_bouts_excluded_coverage": len(bouts) - len(indexed_bouts),
                        "coverage": coverage,
                        "n_metric_rows": len(block_metric_rows),
                        "n_metric_rows_ok": sum(row["status"] == "ok" for row in block_metric_rows),
                        "closure": summarize_closure(block_closure_rows),
                    }
                )

            subject_metric_rows = metric_rows[subject_metric_start:]
            subject_closure_rows = closure_rows[subject_closure_start:]
            source_paths: set[Path] = {manifest_path(subject), mocap_path(subject)}
            source_paths.update(subject_used_raw_paths)
            source_paths.update(
                _resolve_registry_raw(raw)
                for raw in placement.raw_segments
                if _resolve_registry_raw(raw).is_file()
            )
            source_ref_paths: list[Path] = []
            unresolved_source_refs: list[str] = []
            for source_ref in placement.provenance.get("source_refs", []):
                resolved_ref = _resolve_source_ref(str(source_ref))
                if resolved_ref is None:
                    unresolved_source_refs.append(str(source_ref))
                else:
                    source_ref_paths.append(resolved_ref)
            source_paths.update(source_ref_paths)

            payload = {
                "schema_version": SCHEMA_VERSION,
                "run_type": "corrected_regional_quaternion_qc",
                "trial_id": tid,
                "placement": placement.provenance_record(),
                "summary": {
                    "n_blocks": len(block_summaries),
                    "n_bouts_detected": sum(block["n_bouts_detected"] for block in block_summaries),
                    "n_bouts_scored": sum(block["n_bouts_scored"] for block in block_summaries),
                    "n_metric_rows": len(subject_metric_rows),
                    "n_metric_rows_ok": sum(row["status"] == "ok" for row in subject_metric_rows),
                    "relations": summarize_relations(subject_metric_rows),
                    "closure": summarize_closure(subject_closure_rows),
                },
                "blocks": block_summaries,
                "source_hashes": [file_record(path) for path in sorted(source_paths) if path.is_file()],
                "unresolved_registry_source_refs": unresolved_source_refs,
            }
            subject_payloads[tid] = payload
            subject_out = out_dir / "subjects" / f"{tid}.json"
            subject_out.parent.mkdir(parents=True, exist_ok=True)
            subject_out.write_text(
                json.dumps(_jsonable(payload), indent=2, ensure_ascii=False) + "\n",
                encoding="utf-8",
            )
            subject_json_paths.append(subject_out)
            print(
                f"{tid}: {payload['summary']['n_blocks']} blocks, "
                f"{payload['summary']['n_bouts_scored']}/"
                f"{payload['summary']['n_bouts_detected']} bouts scored, "
                f"{len(subject_metric_rows)} regional rows"
            )
    finally:
        mlb.fusion_args = original_fusion_args

    metrics_out = out_dir / "regional_bout_metrics.csv"
    closure_out = out_dir / "regional_chain_closure.csv"
    _write_csv(metrics_out, METRIC_FIELDS, metric_rows)
    _write_csv(closure_out, CLOSURE_FIELDS, closure_rows)

    cohort = {
        "schema_version": SCHEMA_VERSION,
        "run_type": "corrected_regional_quaternion_qc",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "subjects": list(subject_payloads),
        "summary": {
            "n_subjects": len(subjects),
            "n_blocks": sum(payload["summary"]["n_blocks"] for payload in subject_payloads.values()),
            "n_bouts_detected": sum(payload["summary"]["n_bouts_detected"] for payload in subject_payloads.values()),
            "n_bouts_scored": sum(payload["summary"]["n_bouts_scored"] for payload in subject_payloads.values()),
            "n_metric_rows": len(metric_rows),
            "n_metric_rows_ok": sum(row["status"] == "ok" for row in metric_rows),
            "n_closure_rows": len(closure_rows),
            "relations": summarize_relations(metric_rows),
            "closure": summarize_closure(closure_rows),
        },
        "subject_summaries": {
            tid: {
                "json": f"subjects/{tid}.json",
                **payload["summary"],
            }
            for tid, payload in subject_payloads.items()
        },
    }
    cohort_out = out_dir / "cohort_regional_qc.json"
    cohort_out.write_text(
        json.dumps(_jsonable(cohort), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )

    dependency_paths: set[Path] = {
        Path(__file__).resolve(),
        registry_path,
        HERE / "dataset_adapter.py",
        Path(mlb.__file__).resolve(),
        Path(fiv.__file__).resolve(),
        Path(pm.__file__).resolve(),
        Path(pf.__file__).resolve(),
        Path(t06.__file__).resolve(),
        Path(sd.__file__).resolve(),
        HERE / "session_recipe.py",
        HERE / "coverage_gate.py",
        quality_manifest_path,
        *[manifest_path(subject) for subject in subjects],
        *[mocap_path(subject) for subject in subjects],
        *all_used_raw_paths,
    }
    for placement in placements.values():
        dependency_paths.update(
            _resolve_registry_raw(raw)
            for raw in placement.raw_segments
            if _resolve_registry_raw(raw).is_file()
        )
        dependency_paths.update(
            resolved
            for source_ref in placement.provenance.get("source_refs", [])
            if (resolved := _resolve_source_ref(str(source_ref))) is not None
        )

    output_paths = [metrics_out, closure_out, cohort_out, *subject_json_paths]
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "run_type": "corrected_regional_quaternion_qc",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "command": [sys.executable, str(Path(__file__).resolve()), *command_args],
        "environment": {
            "python": platform.python_version(),
            "executable": sys.executable,
            "platform": platform.platform(),
            "numpy": np.__version__,
        },
        "output_policy": {
            "shadow_only": True,
            "out_dir": str(out_dir),
            "data_clean_rejected": True,
            "new_or_empty_required": True,
            "data_clean_untouched_by_design": True,
        },
        "mapping_registry": file_record(registry_path),
        "placements": {
            tid: placement.provenance_record() for tid, placement in placements.items()
        },
        "frozen_authority": {
            "subject_block_iterator": "dataset_adapter.subject_blocks",
            "clocks_windows_bouts": "reused unchanged from subject_blocks",
            "detector": "session_recipe.peak_bouts as called by the frozen iterator",
            "detector_repair": False,
            "coverage_gate": (
                "movement and fixed pre-neutral both require >=95% local timestamp support "
                "and max_gap <= 2x nearest-sample tolerance"
            ),
        },
        "block_quality": {
            "source": file_record(quality_manifest_path),
            "run_type": quality_payload.get("run_type"),
            "readout_policy_version": quality_payload.get("readout_policy_version"),
            "quality_policy_version": quality_payload.get("quality_policy_version"),
        },
        "relation_policy": {
            "posterior_spinal_regions": [
                "sacrum_to_lower",
                "lower_to_mid",
                "mid_to_upper",
            ],
            "thorax_crosscheck_not_spinal_region": "upper_to_sternum",
            "whole_back_direct": "sacrum_to_upper",
            "whole_trunk_direct": "sacrum_to_sternum",
            "scalar_chain_sum": "forbidden",
            "curvature_claim_boundary": (
                "neutral-tared regional relative-angle change proxy; not static curvature and not "
                "curvature per unit length"
            ),
        },
        "metric_policy": {
            "relation_tare": "fixed MoCap [lo-1.2, lo-0.2] mapped by frozen clock; missing neutral coverage rejects the bout",
            "swing_twist": "decompose each locally tared relation independently about SEGMENT_TWIST_AXIS",
            "rom": "per-bout peak-to-peak on finite samples; no relation scalar is added to another",
            "return_to_zero": "post-bout MoCap [hi+0.2, hi+1.2] mapped by the frozen clock; no fallback when unavailable",
            "valid_sample": "finite non-zero-norm q_rel row",
        },
        "closure_policy": {
            "input": "raw q_rel only; no independently tared quaternion and no scalar angle",
            "posterior_equation": "q_SL*q_LM*q_MU == q_SU",
            "full_equation": "q_SL*q_LM*q_MU*q_US == q_S_sternum",
            "tolerance_deg": CLOSURE_TOLERANCE_DEG,
            "interpretation": "implementation/order check only; not independent placement validation",
        },
        "summary": cohort["summary"],
        "sources": [file_record(path) for path in sorted(dependency_paths) if path.is_file()],
        "outputs": [file_record(path) for path in output_paths if path.is_file()],
        "manifest_self_hash_note": "The manifest does not recursively hash itself; every other emitted artifact is hashed above.",
    }
    manifest_out = out_dir / "regional_qc_manifest.json"
    manifest_out.write_text(
        json.dumps(_jsonable(manifest), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )

    print(f"wrote {manifest_out}")
    print(f"wrote {metrics_out} ({len(metric_rows)} rows)")
    print(f"wrote {closure_out} ({len(closure_rows)} rows)")
    print(f"wrote {cohort_out}")
    return 0


def main(argv: list[str] | None = None) -> int:
    raw_args = list(sys.argv[1:] if argv is None else argv)
    args = build_parser().parse_args(raw_args)
    subjects = normalize_subjects(args.subjects)
    out_dir = validate_out_dir(args.out_dir, dry_check=args.dry_check)
    if args.dry_check:
        return dry_check(subjects, args.registry, args.quality_manifest, out_dir)
    return run_regional_qc(
        subjects, args.registry, args.quality_manifest, out_dir, raw_args
    )


if __name__ == "__main__":
    raise SystemExit(main())
