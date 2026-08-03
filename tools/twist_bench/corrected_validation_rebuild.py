from __future__ import annotations

"""Placement-aware, shadow-only rebuild of the 13-subject validation table.

This runner deliberately reuses the frozen subject/block iterator from
``dataset_adapter.py``.  Consequently, clocks, protocol windows and
legacy ``peak_bouts`` behaviour stay fixed while the IMU-to-anatomy mapping is
corrected.  It is an impact-isolation run, not a detector-policy rewrite.

The anatomical readouts are:

* bend: ``sacrum_to_upper`` yaw-masked swing with one uniform per-bout local
  re-tare policy in the default ``corrected_uniform`` mode;
* twist: locally re-tared ``sacrum_to_sternum`` axial twist, matching the
  sternum-vs-S1 MoCap reference.  The old name ``upper_to_sternum`` is not used.

Nothing is ever written under ``data_clean``.  ``--out-dir`` is mandatory and
must be new or empty, so a rebuild cannot silently replace a previous run.
Use ``--dry-check`` to validate paths, registry eligibility and manifest schema
without importing the numerical pipeline or loading any trial data.
"""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import platform
from pathlib import Path
import sys
from typing import Any, Iterable


HERE = Path(__file__).resolve().parent
TB = HERE
DATA_CLEAN = HERE / "data_clean"
DEFAULT_REGISTRY = HERE / "config" / "placement_maps_v1.json"
# Cohort comes from the dataset config, not from a constant here -- a hard-coded
# session list makes the script describe one study rather than one method.
sys.path.insert(0, str(HERE))
import placement_maps as pm  # noqa: E402 -- stdlib-only and safe for --dry-check
from coverage_gate import assess_scoring_coverage  # noqa: E402


METRIC_FIELDS = (
    "n_reps",
    "pooled_r",
    "gain",
    "raw_rmse_deg",
    "raw_acc",
    "calibrated_rmse_deg",
    "heldout_rmse_deg",
    "heldout_acc",
    "rom_deg",
    "per_rep_r_median",
    "per_rep_r_min",
    "accuracy_basis",
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
        help="Subject ids such as 02 03 or T02 T03 (default: all 13 usable subjects).",
    )
    parser.add_argument(
        "--analysis-mode",
        choices=("mapping_only", "corrected_uniform"),
        default="corrected_uniform",
        help=(
            "corrected_uniform uses one corrected readout policy and rebuilds quality labels; "
            "mapping_only preserves legacy bend selection solely for impact isolation"
        ),
    )
    parser.add_argument(
        "--dry-check",
        action="store_true",
        help="Validate registry/manifests/output safety only; write nothing and load no raw data.",
    )
    return parser


def normalize_subjects(values: Iterable[str]) -> list[str]:
    out: list[str] = []
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
        if subject not in out:
            out.append(subject)
    if not out:
        raise SystemExit("--subjects resolved to an empty cohort")
    return out


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


def manifest_path(subject: str) -> Path:
    return DATA_CLEAN / f"T{subject}_P{subject}" / "block_manifest.json"


def mocap_path(subject: str) -> Path:
    return HERE / "data" / "mocap" / f"P{subject}_all.csv"


def trial_id(subject: str) -> str:
    return f"T{subject}_P{subject}"


def _manifest_blocks(manifest: dict[str, Any]) -> list[tuple[str, dict[str, Any]]]:
    blocks = manifest.get("blocks")
    if isinstance(blocks, list):
        out = []
        for index, block in enumerate(blocks):
            if not isinstance(block, dict):
                raise ValueError(f"blocks[{index}] is not an object")
            block_id = block.get("id") or block.get("block") or block.get("block_id")
            if not block_id:
                raise ValueError(f"blocks[{index}] has no id")
            out.append((str(block_id), block))
        return out
    if isinstance(blocks, dict):
        if not all(isinstance(value, dict) for value in blocks.values()):
            raise ValueError("one or more mapping-schema blocks are not objects")
        return [(str(key), value) for key, value in blocks.items()]
    raise ValueError("manifest.blocks must be a list or object")


def _resolve_registry_raw(raw: str) -> Path:
    candidate = Path(raw)
    if candidate.is_absolute():
        return candidate
    return HERE / candidate


def dry_check(subjects: list[str], registry_path: Path, out_dir: Path) -> int:
    registry_path = registry_path.expanduser().resolve()
    registry = pm.load_placement_registry(registry_path)
    errors: list[str] = []
    for subject in subjects:
        tid = trial_id(subject)
        try:
            placement = pm.resolve_placement(trial_id=tid, config_path=registry_path)
        except Exception as exc:  # registry policy failures should be reported together
            errors.append(f"{tid}: placement: {exc}")
            continue
        if placement.trial_id not in registry:
            errors.append(f"{tid}: placement vanished after eager registry validation")
        man_path = manifest_path(subject)
        moc_path = mocap_path(subject)
        if not man_path.is_file():
            errors.append(f"{tid}: missing legacy manifest {man_path}")
        else:
            try:
                manifest = json.loads(man_path.read_text(encoding="utf-8"))
                blocks = _manifest_blocks(manifest)
                if not blocks:
                    errors.append(f"{tid}: manifest has no blocks")
            except Exception as exc:
                errors.append(f"{tid}: invalid manifest: {exc}")
        if not moc_path.is_file():
            errors.append(f"{tid}: missing MoCap {moc_path}")
        missing_raw = [
            str(_resolve_registry_raw(raw))
            for raw in placement.raw_segments
            if not _resolve_registry_raw(raw).is_file()
        ]
        if missing_raw:
            errors.append(f"{tid}: registry raw paths missing: {missing_raw}")

    if errors:
        raise SystemExit("dry-check failed:\n- " + "\n- ".join(errors))
    print(
        f"dry-check OK: {len(subjects)} subjects, registry={registry_path}, "
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
        "size_bytes": resolved.stat().st_size,
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
    if hasattr(value, "item"):
        try:
            return value.item()
        except (TypeError, ValueError):
            pass
    return value


def _number(value: Any) -> float | int | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return value
    return None


def _unwrap_metric(metric: dict[str, Any]) -> dict[str, Any]:
    current = metric
    seen: set[int] = set()
    while isinstance(current, dict) and isinstance(current.get("metric"), dict):
        if id(current) in seen:
            break
        seen.add(id(current))
        current = current["metric"]
    return current


def normalize_metric(metric: dict[str, Any] | None) -> dict[str, Any]:
    if not isinstance(metric, dict):
        return {}
    metric = _unwrap_metric(metric)
    pooled_r = metric.get("pooled_r", metric.get("r", metric.get("r_raw", metric.get("abs_r"))))
    # Do not relabel an in-sample calibrated RMSE as held-out.  Historical
    # ``rmse_after_gain_deg`` values stay in the calibrated field unless the
    # source explicitly names a held-out metric.
    heldout_rmse = metric.get("heldout_rmse_deg")
    calibrated_rmse = metric.get(
        "calibrated_rmse_deg", metric.get("rmse_after_gain_deg")
    )
    rom = metric.get("rom_deg", metric.get("mocap_rom_deg"))
    normalized = {
        "n_reps": metric.get("n_reps"),
        "pooled_r": pooled_r,
        "gain": metric.get("gain", metric.get("gain_after_sign", metric.get("gain_raw"))),
        "raw_rmse_deg": metric.get("raw_rmse_deg"),
        "raw_acc": metric.get("raw_acc"),
        "calibrated_rmse_deg": calibrated_rmse,
        "heldout_rmse_deg": heldout_rmse,
        "heldout_acc": metric.get("heldout_acc"),
        "rom_deg": rom,
        "per_rep_r_median": metric.get("per_rep_r_median"),
        "per_rep_r_min": metric.get("per_rep_r_min"),
        "accuracy_basis": metric.get("accuracy_basis"),
    }
    return {key: value for key, value in normalized.items() if value is not None}


def legacy_metric(block: dict[str, Any]) -> tuple[str, dict[str, Any]]:
    """Select the most headline-like metric present in each historical schema."""
    if isinstance(block.get("zero_lag"), dict):
        return "zero_lag", block["zero_lag"]

    direct_keys = {
        "n_reps", "pooled_r", "r", "gain", "raw_rmse_deg", "raw_acc",
        "calibrated_rmse_deg", "heldout_rmse_deg", "heldout_acc", "rom_deg",
        "per_rep_r_median", "per_rep_r_min", "accuracy_basis",
    }
    if direct_keys.intersection(block):
        return "block_metric", {key: block[key] for key in block if key in direct_keys}

    for key in (
        "heldout_metric",
        "twist_functional_retare_metric",
        "sflp_yawmasked_swing",
        "diagnostic_metric",
    ):
        if isinstance(block.get(key), dict):
            return key, block[key]
    return "missing", {}


def legacy_readout(block: dict[str, Any]) -> tuple[str | None, str]:
    for key in ("selected_readout", "accepted_readout", "readout", "mode"):
        value = block.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip(), key
    return None, "missing_default_global"


def compare_metrics(corrected: dict[str, Any], legacy: dict[str, Any]) -> dict[str, Any]:
    deltas: dict[str, Any] = {}
    for field in METRIC_FIELDS:
        new = _number(corrected.get(field))
        old = _number(legacy.get(field))
        if new is not None and old is not None:
            deltas[field] = round(float(new) - float(old), 6)
    return deltas


def score_uniform_zero_lag(
    *,
    res: Any,
    a: float,
    b: float,
    tm: Any,
    signal: Any,
    bouts: list[tuple[float, float]],
    series_factory: Any,
    abs_mocap: bool,
    np: Any,
) -> dict[str, Any]:
    """Pre-registered equal-bout, zero-lag score with true leave-one-bout-out."""

    xs: list[Any] = []
    ys: list[Any] = []
    per_rep_r: list[float] = []
    for lo, hi in bouts:
        series = series_factory(lo)
        grid = np.linspace(float(lo), float(hi), 200, endpoint=False)
        pre = np.linspace(float(lo - 1.2), float(lo - 0.2), 100, endpoint=False)
        mocap_zero = float(np.mean(np.interp(pre, tm, signal)))
        imu_zero = float(np.mean(np.interp(a * pre + b, res.t_s, series)))
        y = np.interp(grid, tm, signal) - mocap_zero
        x = np.interp(a * grid + b, res.t_s, series) - imu_zero
        if abs_mocap:
            y = np.abs(y)
        if not (np.all(np.isfinite(x)) and np.all(np.isfinite(y))):
            continue
        xs.append(x)
        ys.append(y)
        if np.std(x) >= 1e-6 and np.std(y) >= 1e-6:
            per_rep_r.append(float(np.corrcoef(x, y)[0, 1]))

    if not xs:
        return {"n_reps": 0, "scoring_policy": "uniform_equal_bout_zero_lag_loro_v1"}
    x_all = np.concatenate(xs)
    y_all = np.concatenate(ys)
    gain, intercept = np.polyfit(x_all, y_all, 1)
    pred = gain * x_all + intercept
    rom = float(np.ptp(y_all))
    raw_rmse = float(np.sqrt(np.mean((y_all - x_all) ** 2)))
    out: dict[str, Any] = {
        "n_reps": len(xs),
        "pooled_r": round(float(np.corrcoef(x_all, y_all)[0, 1]), 3),
        "gain": round(float(gain), 3),
        "raw_rmse_deg": round(raw_rmse, 2),
        "raw_acc": round(1.0 - raw_rmse / max(rom, 1e-6), 3),
        "calibrated_rmse_deg": round(float(np.sqrt(np.mean((y_all - pred) ** 2))), 2),
        "rom_deg": round(rom, 1),
        "per_rep_r_median": round(float(np.median(per_rep_r)), 3) if per_rep_r else None,
        "per_rep_r_min": round(float(np.min(per_rep_r)), 3) if per_rep_r else None,
        "lag_median_s": 0.0,
        "scoring_policy": "uniform_equal_bout_zero_lag_loro_v1",
    }
    native_scale = 0.7 <= float(gain) <= 1.4
    out["accuracy_basis"] = (
        "native" if float(out["raw_acc"]) >= 0.6 and native_scale else "gain_corrected_only"
    )
    if len(xs) >= 2:
        fold_rmse: list[float] = []
        fold_acc: list[float] = []
        gains: list[float] = []
        for k in range(len(xs)):
            train_x = np.concatenate([xs[i] for i in range(len(xs)) if i != k])
            train_y = np.concatenate([ys[i] for i in range(len(ys)) if i != k])
            fold_gain, fold_intercept = np.polyfit(train_x, train_y, 1)
            prediction = fold_gain * xs[k] + fold_intercept
            rmse_k = float(np.sqrt(np.mean((ys[k] - prediction) ** 2)))
            rom_k = float(np.ptp(ys[k]))
            fold_rmse.append(rmse_k)
            fold_acc.append(1.0 - rmse_k / max(rom_k, 1e-6))
            gains.append(float(fold_gain))
        out["heldout_rmse_deg"] = round(
            float(np.sqrt(np.mean(np.square(fold_rmse)))), 2
        )
        out["heldout_acc"] = round(float(np.mean(fold_acc)), 3)
        out["per_bout_heldout_rmse_deg"] = [round(value, 4) for value in fold_rmse]
        out["per_bout_heldout_acc"] = [round(value, 4) for value in fold_acc]
        out["gain_range"] = [round(float(np.min(gains)), 3), round(float(np.max(gains)), 3)]
    return out


def score_corrected_block(
    *,
    block_id: str,
    label: str,
    block: dict[str, Any],
    res: Any,
    a: float,
    b: float,
    tm: Any,
    signal: Any,
    bouts: list[tuple[float, float]],
    t06: Any,
    t09: Any,
    analysis_mode: str,
    np: Any,
) -> tuple[str, str, dict[str, Any]]:
    clock = {"a": float(a), "b": float(b)}
    old_readout, selection_source = legacy_readout(block)
    if label in {"left_twist", "right_twist"}:
        readout = "local_retare_sacrum_to_sternum_twist"
        factory = lambda lo, result=res, aa=float(a), bb=float(b): t06.local_twist(result, aa, bb, lo)
        score = (
            score_uniform_zero_lag(
                res=res, a=float(a), b=float(b), tm=tm, signal=signal, bouts=bouts,
                series_factory=factory, abs_mocap=False, np=np,
            )
            if analysis_mode == "corrected_uniform"
            else t06.score_block(res, clock, tm, signal, bouts, factory, False, False)
        )
        return readout, f"corrected_anatomical_target;legacy={old_readout}", score

    use_local = analysis_mode == "corrected_uniform" or (
        old_readout is not None
        and "local" in old_readout.lower()
        and "swing" in old_readout.lower()
    )
    if use_local:
        readout = "local_retare_yawmasked_sacrum_to_upper_swing"
        factory = lambda lo, result=res, aa=float(a), bb=float(b): t06.local_swing(result, aa, bb, lo)
    else:
        readout = "global_yawmasked_sacrum_to_upper_swing"
        factory = lambda lo, result=res: result.relations["sacrum_to_upper"].swing_deg
    score = (
        score_uniform_zero_lag(
            res=res, a=float(a), b=float(b), tm=tm, signal=signal, bouts=bouts,
            series_factory=factory, abs_mocap=True, np=np,
        )
        if analysis_mode == "corrected_uniform"
        else t06.score_block(res, clock, tm, signal, bouts, factory, True, False)
    )
    score = t09.mark_bend_gain_corrected(score)
    source = (
        "uniform_corrected_policy"
        if analysis_mode == "corrected_uniform"
        else f"{selection_source}:{old_readout or 'global_fallback'}"
    )
    return readout, source, score


def corrected_quality(
    metric: dict[str, Any], n_scored: int, n_detected: int
) -> tuple[str, list[str]]:
    """Outcome-independent schema using only corrected coverage and zero-lag metrics."""

    if n_scored == 0 or n_detected == 0:
        return "invalid_no_imu_coverage", ["insufficient_coverage"]
    reasons: list[str] = []
    coverage = float(n_scored / n_detected)
    if coverage < 0.90:
        reasons.append("clean_insufficient_coverage")
    if n_scored < 10:
        reasons.append("clean_insufficient_n")
    if metric.get("heldout_rmse_deg") is None or metric.get("heldout_acc") is None:
        return "invalid_insufficient_repetitions", reasons + ["missing_true_heldout"]
    corr = float(metric.get("pooled_r", -1.0))
    rmse = float(metric.get("heldout_rmse_deg", float("inf")))
    acc = float(metric.get("heldout_acc", -1.0))
    if coverage >= 0.90 and n_scored >= 10 and corr >= 0.85 and rmse <= 3.0 and acc >= 0.75:
        return "clean", []
    if corr < 0.85:
        reasons.append("clean_zero_lag_shape_failure")
    if rmse > 3.0:
        reasons.append("clean_heldout_absolute_error_failure")
    if acc < 0.75:
        reasons.append("clean_heldout_relative_accuracy_failure")
    if coverage >= 0.70 and n_scored >= 5 and corr >= 0.70 and rmse <= 5.0 and acc >= 0.50:
        return "low_conf", reasons
    if coverage < 0.70:
        reasons.append("insufficient_coverage")
    if n_scored < 5:
        reasons.append("insufficient_n")
    if corr < 0.70:
        reasons.append("zero_lag_shape_failure")
    if rmse > 5.0:
        reasons.append("heldout_absolute_error_failure")
    if acc < 0.50:
        reasons.append("heldout_relative_accuracy_failure")
    return "invalid_quality_gate", reasons


def _median(values: list[float]) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return round(float(ordered[middle]), 6)
    return round(float((ordered[middle - 1] + ordered[middle]) / 2.0), 6)


def cohort_summary(rows: list[dict[str, Any]], subjects: list[str]) -> dict[str, Any]:
    rs = [float(row["corrected_metric"]["pooled_r"]) for row in rows if _number(row["corrected_metric"].get("pooled_r")) is not None]
    rmses = [
        float(row["corrected_metric"]["heldout_rmse_deg"])
        for row in rows
        if _number(row["corrected_metric"].get("heldout_rmse_deg")) is not None
    ]
    return {
        "n_subjects": len(subjects),
        "subjects": [f"T{subject}" for subject in subjects],
        "n_blocks": len(rows),
        "n_bouts_detected": sum(int(row["n_bouts_detected"]) for row in rows),
        "n_bouts_scored": sum(int(row["n_bouts_scored"]) for row in rows),
        "n_bouts_excluded_coverage": sum(
            int(row["n_bouts_excluded_coverage"]) for row in rows
        ),
        "median_corrected_pooled_r": _median(rs),
        "median_corrected_heldout_rmse_deg": _median(rmses),
        "blocks_by_subject": {
            f"T{subject}": sum(row["subject"] == f"T{subject}" for row in rows)
            for subject in subjects
        },
    }


def csv_row(row: dict[str, Any]) -> dict[str, Any]:
    corrected = row["corrected_metric"]
    legacy = row["legacy_metric_normalized"]
    deltas = row["delta_corrected_minus_legacy"]
    flat: dict[str, Any] = {
        "subject": row["subject"],
        "trial_id": row["trial_id"],
        "block": row["block"],
        "label": row["label"],
        "quality": row["quality"],
        "quality_reasons": ";".join(row["quality_reasons"]),
        "mapping_version": row["mapping_version"],
        "mapping_sha256": row["mapping_sha256"],
        "corrected_readout": row["corrected_readout"],
        "selection_source": row["selection_source"],
        "n_bouts_detected": row["n_bouts_detected"],
        "n_bouts_scored": row["n_bouts_scored"],
        "n_bouts_excluded_coverage": row["n_bouts_excluded_coverage"],
        "block_coverage_fraction": (
            round(row["n_bouts_scored"] / row["n_bouts_detected"], 6)
            if row["n_bouts_detected"]
            else 0.0
        ),
        "legacy_metric_source": row["legacy_metric_source"],
        "legacy_status": row["legacy_status"],
    }
    for field in METRIC_FIELDS:
        flat[f"corrected_{field}"] = corrected.get(field)
        flat[f"legacy_{field}"] = legacy.get(field)
        flat[f"delta_{field}"] = deltas.get(field)
    return flat


def run_rebuild(
    subjects: list[str], registry_path: Path, out_dir: Path, analysis_mode: str
) -> int:
    # Delayed imports keep --dry-check cheap and independent of numerical dependencies.
    import dataset_adapter as mlb  # type: ignore  # noqa: PLC0415
    import five_imu_fusion as fiv  # noqa: PLC0415
    import session_recipe as t06  # noqa: PLC0415
    import session_recipe as t09  # noqa: PLC0415

    registry_path = registry_path.expanduser().resolve()
    placements = {
        trial_id(subject): pm.resolve_placement(
            trial_id=trial_id(subject), config_path=registry_path
        ).provenance_record()
        for subject in subjects
    }
    run_type = (
        "corrected_uniform_validation"
        if analysis_mode == "corrected_uniform"
        else "corrected_mapping_only_validation"
    )

    # dataset_adapter.subject_blocks is the frozen clock/window/bout authority.  Its
    # loader delegates through fusion_args, so replace only that resolution hook
    # for the duration of this single-threaded run to honour --registry.
    original_fusion_args = mlb.fusion_args

    def configured_fusion_args(subject: str, imu: Path, filter_name: str):
        placement = pm.resolve_placement(
            trial_id=trial_id(subject),
            raw_path=imu,
            config_path=registry_path,
        )
        return fiv.make_args(filter=filter_name, **placement.fusion_kwargs())

    mlb.fusion_args = configured_fusion_args
    all_rows: list[dict[str, Any]] = []
    subject_payloads: dict[str, dict[str, Any]] = {}
    used_raw_paths: set[Path] = set()
    try:
        for subject in subjects:
            tid = trial_id(subject)
            placement = pm.resolve_placement(trial_id=tid, config_path=registry_path)
            tm, flex, lat, axial = mlb.sd.mocap_signed(mocap_path(subject))
            signals = {"flex": flex, "lat": lat, "axial": axial}
            rows: list[dict[str, Any]] = []

            for block_id, block, res, a, b, bouts in mlb.subject_blocks(subject, tm, signals):
                override = mlb.BLOCK_OVERRIDES.get(subject, {}).get(block_id, {})
                label = mlb._canon_label(override.get("label", block["label"]))
                if label not in mlb.LABELS:
                    continue
                signal_key = override.get(
                    "sig",
                    block.get("mocap_signal", block.get("primary_signal", mlb.SIG_BY_LABEL[label])),
                )
                scored_bouts, bout_coverage = assess_scoring_coverage(
                    res.t_s, float(a), float(b), bouts, mlb.np
                )
                readout, selection_source, corrected_raw = score_corrected_block(
                    block_id=block_id,
                    label=label,
                    block=block,
                    res=res,
                    a=a,
                    b=b,
                    tm=tm,
                    signal=signals[signal_key],
                    bouts=scored_bouts,
                    t06=t06,
                    t09=t09,
                    analysis_mode=analysis_mode,
                    np=mlb.np,
                )
                corrected = normalize_metric(corrected_raw)
                legacy_source, legacy_raw = legacy_metric(block)
                legacy_normalized = normalize_metric(legacy_raw)
                legacy_status = block.get("verdict", block.get("status"))
                source_input = Path(str(res.summary.get("input", "")))
                if source_input.is_file():
                    used_raw_paths.add(source_input.resolve())
                legacy_quality = mlb.quality_for(subject, block_id, block)
                if analysis_mode == "corrected_uniform":
                    quality, quality_reasons = corrected_quality(
                        corrected, len(scored_bouts), len(bouts)
                    )
                else:
                    quality, quality_reasons = legacy_quality, ["legacy_mapping_only_quality"]
                row = {
                    "subject": f"T{subject}",
                    "trial_id": tid,
                    "block": block_id,
                    "label": label,
                    "quality": quality,
                    "quality_reasons": quality_reasons,
                    "legacy_quality": legacy_quality,
                    "mapping_version": placement.mapping_version,
                    "mapping_sha256": placement.canonical_sha256,
                    "corrected_readout": readout,
                    "selection_source": selection_source,
                    "clock": {"a": float(a), "b": float(b)},
                    "detected_bouts": [[float(lo), float(hi)] for lo, hi in bouts],
                    "scored_bouts": [[float(lo), float(hi)] for lo, hi in scored_bouts],
                    "n_bouts_detected": len(bouts),
                    "n_bouts_scored": len(scored_bouts),
                    "n_bouts_excluded_coverage": len(bouts) - len(scored_bouts),
                    "bout_coverage": bout_coverage,
                    "corrected_metric": corrected,
                    "corrected_metric_full": _jsonable(corrected_raw),
                    "legacy_metric_source": legacy_source,
                    "legacy_metric_normalized": legacy_normalized,
                    "legacy_metric_full": _jsonable(legacy_raw),
                    "legacy_status": legacy_status,
                    "delta_corrected_minus_legacy": compare_metrics(corrected, legacy_normalized),
                }
                rows.append(row)
                all_rows.append(row)

            source_paths = [manifest_path(subject), mocap_path(subject)]
            for raw in placement.raw_segments:
                path = _resolve_registry_raw(raw)
                if path.is_file():
                    source_paths.append(path)
            payload = {
                "trial_id": tid,
                "run_type": run_type,
                "placement": placements[tid],
                "legacy_manifest": {
                    "path": str(manifest_path(subject).resolve()),
                    "status": json.loads(manifest_path(subject).read_text(encoding="utf-8")).get("status"),
                },
                "readout_policy": {
                    "bend": (
                        "local_retare_sacrum_to_upper_swing for every block"
                        if analysis_mode == "corrected_uniform"
                        else "sacrum_to_upper; preserve recorded global/local selection"
                    ),
                    "twist": "local_retare_sacrum_to_sternum_twist",
                    "lag_search": False,
                },
                "blocks": rows,
                "source_hashes": [file_record(path) for path in dict.fromkeys(source_paths)],
            }
            subject_payloads[tid] = payload
            subject_out = out_dir / "subjects" / f"{tid}.json"
            subject_out.parent.mkdir(parents=True, exist_ok=True)
            subject_out.write_text(json.dumps(_jsonable(payload), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
            print(
                f"{tid}: {len(rows)} blocks, "
                f"{sum(row['n_bouts_scored'] for row in rows)}/"
                f"{sum(row['n_bouts_detected'] for row in rows)} bouts scored"
            )
    finally:
        mlb.fusion_args = original_fusion_args

    placements_out = out_dir / "placements.json"
    placements_out.write_text(
        json.dumps(_jsonable(placements), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )

    block_quality_payload = {
        "schema_version": 1,
        "run_type": (
            "corrected_uniform_block_quality"
            if analysis_mode == "corrected_uniform"
            else "legacy_mapping_only_block_quality"
        ),
        "mapping_registry": str(registry_path),
        "mapping_registry_sha256": sha256_file(registry_path),
        "readout_policy_version": (
            "corrected_uniform_local_v1"
            if analysis_mode == "corrected_uniform"
            else "legacy_selection_mapping_only"
        ),
        "quality_policy_version": (
            "timestamp95_gap2tol_coverage90_n10_r85_loro3_acc75_v2"
            if analysis_mode == "corrected_uniform"
            else "legacy_quality_mapping_only"
        ),
        "rows": [
            {
                "subject": row["subject"],
                "trial_id": row["trial_id"],
                "block": row["block"],
                "label": row["label"],
                "mapping_sha256": row["mapping_sha256"],
                "n_expected": row["n_bouts_detected"],
                "n_covered": row["n_bouts_scored"],
                "n_scored": row["n_bouts_scored"],
                "coverage": (
                    round(row["n_bouts_scored"] / row["n_bouts_detected"], 6)
                    if row["n_bouts_detected"]
                    else 0.0
                ),
                "pooled_r": row["corrected_metric"].get("pooled_r"),
                "heldout_rmse_deg": row["corrected_metric"].get("heldout_rmse_deg"),
                "heldout_acc": row["corrected_metric"].get("heldout_acc"),
                "quality": row["quality"],
                "quality_reasons": row["quality_reasons"],
            }
            for row in all_rows
        ],
    }
    block_quality_out = out_dir / "block_quality_v1.json"
    block_quality_out.write_text(
        json.dumps(_jsonable(block_quality_payload), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )

    cohort = {
        "schema_version": 1,
        "run_type": run_type,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "mapping_registry": str(registry_path),
        "mapping_registry_sha256": sha256_file(registry_path),
        "method": {
            "block_authority": "dataset_adapter.subject_blocks",
            "detector_policy": "frozen clocks/windows/peak_bouts; strict local IMU coverage gate",
            "bend_relation": "sacrum_to_upper",
            "bend_selection": (
                "uniform per-bout local re-tare"
                if analysis_mode == "corrected_uniform"
                else "recorded legacy global/local readout; global fallback when absent"
            ),
            "twist_relation": "sacrum_to_sternum",
            "twist_tare": "per-bout pre-neutral local re-tare",
            "lag_search": False,
            "output_policy": "shadow-only; data_clean untouched",
            "quality_policy": (
                "recomputed from corrected coverage, n, zero-lag r and explicit held-out RMSE"
                if analysis_mode == "corrected_uniform"
                else "legacy quality retained for mapping-only impact isolation"
            ),
        },
        "summary": cohort_summary(all_rows, subjects),
        "placements": placements,
        "subjects": subject_payloads,
        "rows": all_rows,
    }
    cohort_json = out_dir / "cohort_validation.json"
    cohort_json.write_text(
        json.dumps(_jsonable(cohort), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )

    flat_rows = [csv_row(row) for row in all_rows]
    cohort_csv = out_dir / "cohort_validation.csv"
    if flat_rows:
        with cohort_csv.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(flat_rows[0]))
            writer.writeheader()
            writer.writerows(flat_rows)
    else:
        cohort_csv.write_text("", encoding="utf-8")

    dependency_paths = {
        HERE / "corrected_validation_rebuild.py",
        HERE / "coverage_gate.py",
        HERE / "dataset_adapter.py",
        HERE / "five_imu_fusion.py",
        HERE / "placement_maps.py",
        HERE / "session_recipe.py",
        HERE / "session_recipe.py",
        registry_path,
        *[manifest_path(subject) for subject in subjects],
        *[mocap_path(subject) for subject in subjects],
        *used_raw_paths,
    }
    output_paths = {
        placements_out,
        cohort_json,
        cohort_csv,
        block_quality_out,
        *list((out_dir / "subjects").glob("*.json")),
    }
    provenance = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "command": [
            sys.executable,
            str(HERE / "corrected_validation_rebuild.py"),
            "--analysis-mode",
            analysis_mode,
            "--out-dir",
            str(out_dir),
        ],
        "environment": {
            "python": platform.python_version(),
            "executable": sys.executable,
            "platform": platform.platform(),
            "numpy": getattr(mlb.np, "__version__", "unknown"),
        },
        "sources": [file_record(path) for path in sorted(dependency_paths) if path.is_file()],
        "outputs": [file_record(path) for path in sorted(output_paths) if path.is_file()],
        "data_clean_untouched_by_design": True,
    }
    provenance_out = out_dir / "provenance.json"
    provenance_out.write_text(
        json.dumps(_jsonable(provenance), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(f"wrote {cohort_json}")
    print(f"wrote {cohort_csv}")
    print(f"wrote {block_quality_out}")
    print(f"wrote {provenance_out}")
    return 0


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    subjects = normalize_subjects(args.subjects)
    out_dir = validate_out_dir(args.out_dir, dry_check=args.dry_check)
    if args.dry_check:
        return dry_check(subjects, args.registry, out_dir)
    return run_rebuild(subjects, args.registry, out_dir, args.analysis_mode)


if __name__ == "__main__":
    raise SystemExit(main())
