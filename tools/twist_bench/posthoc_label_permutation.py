#!/usr/bin/env python3
"""Post-hoc conditional label-permutation check for locked Track A Logistic.

This diagnostic deliberately does not alter the locked Track A archive.  It fixes the
Logistic procedure selected in every original outer fold (L2, C=0.01), permutes labels
independently within each participant, and refits all 13 LOSO folds for every permutation.
It is conditional on the already selected procedure: the inner tuning loop is not rerun.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import sys
import time
import warnings
from importlib import metadata
from pathlib import Path
from typing import Any

for _name in (
    "OMP_NUM_THREADS",
    "OPENBLAS_NUM_THREADS",
    "MKL_NUM_THREADS",
    "VECLIB_MAXIMUM_THREADS",
    "NUMEXPR_NUM_THREADS",
):
    os.environ[_name] = "1"

import joblib
import numpy as np
import pandas as pd

from locked_track_a.core import (
    FittedProcedure,
    PRIMARY_SEED,
    compute_training_weights,
    participant_first_macro_f1,
)


HERE = Path(__file__).resolve().parent
LOCKED_RUN = HERE / "runs" / "locked_track_a_2026-07-21"
DEFAULT_INPUT = HERE / "runs" / "allpairs_2026-07-21" / "features_model.csv"
DEFAULT_OUTPUT = (
    HERE
    / "runs"
    / "posthoc_diagnostics_2026-07-22"
    / "conditional_label_permutation_logistic_B999"
)
FIXED_PARAMS = {"family": "l2", "C": 0.01}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_json(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)


def write_json_atomic(path: Path, value: Any) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def package_version(name: str) -> str:
    try:
        return metadata.version(name)
    except metadata.PackageNotFoundError:
        return "not-installed"


def fit_loso_predictions(
    X: np.ndarray,
    y: np.ndarray,
    subjects: np.ndarray,
) -> tuple[np.ndarray, list[dict[str, Any]]]:
    predictions = np.full(y.shape, -1, dtype=int)
    diagnostics: list[dict[str, Any]] = []
    for outer_subject in sorted(np.unique(subjects)):
        test_mask = subjects == outer_subject
        train_mask = ~test_mask
        if set(subjects[train_mask]) & set(subjects[test_mask]):
            raise AssertionError("outer participant overlap")
        weights = compute_training_weights(subjects[train_mask], y[train_mask], "subject_only")
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            procedure = FittedProcedure("logistic", FIXED_PARAMS, PRIMARY_SEED)
            procedure.fit(X[train_mask], y[train_mask], weights)
            predictions[test_mask] = procedure.predict(X[test_mask])
        diagnostics.append(
            {
                "outer_subject": str(outer_subject),
                "warnings": [
                    {"category": item.category.__name__, "message": str(item.message)}
                    for item in caught
                ],
            }
        )
    if np.any(predictions < 0):
        raise AssertionError("not every row received exactly one outer prediction")
    return predictions, diagnostics


def permutation_worker(
    permutation_index: int,
    master_seed: int,
    X: np.ndarray,
    y_original: np.ndarray,
    subjects: np.ndarray,
) -> dict[str, Any]:
    started = time.perf_counter()
    rng = np.random.default_rng(np.random.SeedSequence([master_seed, permutation_index]))
    y_permuted = y_original.copy()
    for participant in sorted(np.unique(subjects)):
        indices = np.flatnonzero(subjects == participant)
        y_permuted[indices] = rng.permutation(y_permuted[indices])

    try:
        predictions, fold_diagnostics = fit_loso_predictions(X, y_permuted, subjects)
        score, per_participant = participant_first_macro_f1(
            y_permuted, predictions, subjects
        )
        warning_records = [
            {"outer_subject": fold["outer_subject"], **warning}
            for fold in fold_diagnostics
            for warning in fold["warnings"]
        ]
        return {
            "permutation_index": int(permutation_index),
            "status": "ok",
            "participant_first_fixed6_macro_f1": float(score),
            "per_participant": per_participant,
            "warning_count": len(warning_records),
            "warnings": warning_records,
            "failure_type": "",
            "failure_message": "",
            "runtime_seconds": float(time.perf_counter() - started),
        }
    except Exception as error:  # preserve failures rather than silently dropping them
        return {
            "permutation_index": int(permutation_index),
            "status": "failed",
            "participant_first_fixed6_macro_f1": None,
            "per_participant": {},
            "warning_count": 0,
            "warnings": [],
            "failure_type": type(error).__name__,
            "failure_message": str(error),
            "runtime_seconds": float(time.perf_counter() - started),
        }


def load_existing(path: Path) -> dict[int, dict[str, Any]]:
    records: dict[int, dict[str, Any]] = {}
    if not path.exists():
        return records
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        if not line.strip():
            continue
        record = json.loads(line)
        index = int(record["permutation_index"])
        if index in records and records[index] != record:
            raise RuntimeError(f"conflicting checkpoint record at line {line_number}")
        records[index] = record
    return records


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--permutations", type=int, default=999)
    parser.add_argument("--seed", type=int, default=20260721)
    parser.add_argument("--jobs", type=int, default=6)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.permutations < 1 or args.jobs < 1:
        raise ValueError("permutations and jobs must both be positive")

    started = time.perf_counter()
    input_path = args.input.resolve()
    output_dir = args.output_dir.resolve()
    feature_columns_path = (LOCKED_RUN / "p0" / "feature_columns.csv").resolve()
    primary_predictions_path = (LOCKED_RUN / "primary" / "outer_predictions.csv").resolve()
    selected_configs_path = (LOCKED_RUN / "primary" / "selected_configs.csv").resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    feature_columns = pd.read_csv(feature_columns_path)["feature_name"].tolist()
    frame = pd.read_csv(input_path)
    if len(feature_columns) != 130 or not all(name.startswith("pair_") for name in feature_columns):
        raise AssertionError("the frozen predictor set is not exactly 130 pair_* columns")
    if not set(feature_columns).issubset(frame.columns):
        raise AssertionError("frozen feature columns are missing from the input")

    X = frame[feature_columns].to_numpy(dtype=np.float64)
    y_original = frame["y"].to_numpy(dtype=int)
    subjects = frame["subject"].astype(str).to_numpy()
    if X.shape != (1387, 130) or len(np.unique(subjects)) != 13:
        raise AssertionError(f"unexpected cohort shape: X={X.shape}, participants={len(np.unique(subjects))}")

    selected = pd.read_csv(selected_configs_path)
    selected = selected.loc[selected["model"] == "logistic"]
    if len(selected) != 13 or set(selected["selected_config_id"]) != {"logistic:00"}:
        raise AssertionError("the fixed C=0.01 procedure was not selected in every original fold")

    stored = pd.read_csv(primary_predictions_path)
    stored = stored.loc[stored["model"] == "logistic"].sort_values("source_row_index")
    if not np.array_equal(stored["source_row_index"].to_numpy(), np.arange(len(frame))):
        raise AssertionError("stored predictions do not cover source rows exactly once")
    if not np.array_equal(stored["y_true"].to_numpy(dtype=int), y_original):
        raise AssertionError("input labels do not align with stored outer predictions")
    if not np.array_equal(stored["outer_test_subject"].astype(str).to_numpy(), subjects):
        raise AssertionError("input participants do not align with stored outer predictions")

    observed_predictions, observed_diagnostics = fit_loso_predictions(X, y_original, subjects)
    stored_predictions = stored["y_pred"].to_numpy(dtype=int)
    if not np.array_equal(observed_predictions, stored_predictions):
        raise AssertionError("fixed-procedure observed refit does not reproduce primary predictions")
    observed_score, observed_per_participant = participant_first_macro_f1(
        y_original, observed_predictions, subjects
    )

    design = {
        "analysis_id": "spinesense_track_a_posthoc_conditional_label_permutation_2026-07-22",
        "status": "post-hoc diagnostic; not part of the locked primary analysis",
        "input_path": str(input_path),
        "input_sha256": sha256_file(input_path),
        "feature_columns_path": str(feature_columns_path),
        "feature_columns_sha256": sha256_file(feature_columns_path),
        "primary_predictions_path": str(primary_predictions_path),
        "primary_predictions_sha256": sha256_file(primary_predictions_path),
        "selected_configs_path": str(selected_configs_path),
        "selected_configs_sha256": sha256_file(selected_configs_path),
        "n_rows": int(len(frame)),
        "n_participants": int(len(np.unique(subjects))),
        "n_features": int(X.shape[1]),
        "labels": list(range(6)),
        "model": "logistic",
        "fixed_params": FIXED_PARAMS,
        "model_seed": PRIMARY_SEED,
        "weight_scheme": "subject_only",
        "outer_validation": "13-fold participant LOSO",
        "preprocessing": "outer-training-only median imputation, variance threshold, standard scaling",
        "permutation_unit": "labels independently permuted within each participant",
        "permutation_consistency": "one complete pseudo-dataset reused across all 13 outer folds per permutation",
        "statistic": "mean of 13 participant fixed-six-class macro-F1 values",
        "tail": "one-sided, permuted statistic >= observed statistic",
        "p_value_formula": "(1 + exceedances) / (B + 1)",
        "permutations": int(args.permutations),
        "permutation_seed": int(args.seed),
        "conditional_limitation": (
            "The already selected L2 C=0.01 procedure is fixed; inner tuning is not rerun "
            "inside permutations, so this is not a randomization test of the full nested selection procedure."
        ),
    }
    design_sha256 = hashlib.sha256(canonical_json(design).encode("utf-8")).hexdigest()
    manifest_path = output_dir / "manifest.json"
    if manifest_path.exists():
        existing_manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if existing_manifest.get("design_sha256") != design_sha256:
            raise RuntimeError("output directory contains a different permutation design")

    manifest = {
        **design,
        "design_sha256": design_sha256,
        "run_status": "running",
        "python": sys.version,
        "python_executable": sys.executable,
        "platform": platform.platform(),
        "packages": {
            name: package_version(name)
            for name in ("numpy", "pandas", "scipy", "scikit-learn", "joblib")
        },
        "jobs": int(args.jobs),
        "observed_score": float(observed_score),
        "observed_per_participant": observed_per_participant,
        "observed_refit_matches_primary_predictions": True,
        "observed_warning_count": int(
            sum(len(item["warnings"]) for item in observed_diagnostics)
        ),
    }
    write_json_atomic(manifest_path, manifest)

    checkpoint_path = output_dir / "null_scores.jsonl"
    records = load_existing(checkpoint_path)
    pending = [index for index in range(args.permutations) if index not in records]
    print(
        f"START B={args.permutations} pending={len(pending)} jobs={args.jobs} "
        f"observed={observed_score:.12f}",
        flush=True,
    )

    if pending:
        generator = joblib.Parallel(
            n_jobs=args.jobs,
            # Threading avoids platform semaphore limits in restricted environments.
            # Each individual sklearn fit is already constrained to one BLAS/OpenMP thread.
            backend="threading",
            return_as="generator_unordered",
            batch_size=1,
            pre_dispatch="2*n_jobs",
        )(
            joblib.delayed(permutation_worker)(index, args.seed, X, y_original, subjects)
            for index in pending
        )
        with checkpoint_path.open("a", encoding="utf-8") as handle:
            for completed_count, record in enumerate(generator, start=1):
                index = int(record["permutation_index"])
                records[index] = record
                handle.write(canonical_json(record) + "\n")
                handle.flush()
                if completed_count % 50 == 0 or completed_count == len(pending):
                    print(
                        f"CHECKPOINT completed={len(records)}/{args.permutations}",
                        flush=True,
                    )

    ordered = [records[index] for index in range(args.permutations)]
    failures = [record for record in ordered if record["status"] != "ok"]
    if failures:
        failure_frame = pd.DataFrame(failures)
        failure_frame.to_csv(output_dir / "warnings_failures.csv", index=False)
        manifest.update(
            {
                "run_status": "failed",
                "failed_permutations": len(failures),
                "runtime_seconds": float(time.perf_counter() - started),
            }
        )
        write_json_atomic(manifest_path, manifest)
        raise RuntimeError(f"{len(failures)} permutation fits failed; see warnings_failures.csv")

    scores = np.asarray(
        [record["participant_first_fixed6_macro_f1"] for record in ordered],
        dtype=float,
    )
    exceedances = int(np.sum(scores >= observed_score))
    p_value = float((1 + exceedances) / (args.permutations + 1))
    summary = {
        "observed_participant_first_fixed6_macro_f1": float(observed_score),
        "permutations": int(args.permutations),
        "exceedances": exceedances,
        "monte_carlo_p_one_sided_plus_one": p_value,
        "minimum_attainable_p": float(1 / (args.permutations + 1)),
        "null_mean": float(np.mean(scores)),
        "null_sd": float(np.std(scores, ddof=1)),
        "null_min": float(np.min(scores)),
        "null_q025": float(np.quantile(scores, 0.025)),
        "null_median": float(np.median(scores)),
        "null_q975": float(np.quantile(scores, 0.975)),
        "null_max": float(np.max(scores)),
        "warning_count": int(sum(record["warning_count"] for record in ordered)),
        "failed_permutations": 0,
        "runtime_seconds": float(time.perf_counter() - started),
        "interpretation": (
            "Post-hoc conditional label-permutation leakage sanity check; this does not "
            "replace row/fold/source leakage audits or test the full nested tuning procedure."
        ),
    }

    pd.DataFrame(
        {
            "permutation_index": np.arange(args.permutations, dtype=int),
            "participant_first_fixed6_macro_f1": scores,
            "exceeds_or_equals_observed": scores >= observed_score,
            "runtime_seconds": [record["runtime_seconds"] for record in ordered],
            "warning_count": [record["warning_count"] for record in ordered],
        }
    ).to_csv(output_dir / "null_scores.csv", index=False)
    pd.DataFrame(
        [
            {
                "permutation_index": record["permutation_index"],
                "outer_subject": warning["outer_subject"],
                "category": warning["category"],
                "message": warning["message"],
            }
            for record in ordered
            for warning in record["warnings"]
        ],
        columns=["permutation_index", "outer_subject", "category", "message"],
    ).to_csv(output_dir / "warnings_failures.csv", index=False)
    write_json_atomic(output_dir / "summary.json", summary)
    manifest.update({"run_status": "complete", "summary": summary})
    write_json_atomic(manifest_path, manifest)
    print(canonical_json(summary), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
