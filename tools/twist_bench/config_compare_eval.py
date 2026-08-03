"""Section 7.5 sensor-configuration comparison: nested-LOSO evaluation of all 31 configurations.

The analysis protocol is inherited from locked Track A and is not re-implemented here: the
estimators, preprocessing pipeline, parameter grids, tie-break selection rule, participant
metrics, and bootstrap helpers are imported from ``locked_track_a.core``.  This module supplies
only the outer/inner orchestration over sensor configurations, the integrity gates, and the
reporting artefacts.

Run order
---------
1. ``config_compare_plan.py`` freezes ``run_plan.json`` (read-only) plus its hash sidecar.
2. This script verifies the plan, verifies the data, then runs the five-sensor configuration
   FIRST and compares it against the Track A archive row by row, fold by fold, and participant
   by participant.  A mismatch aborts before any other configuration is fitted.
3. The remaining 30 configurations run for both models, checkpointed per
   (model, configuration, outer participant) so an interrupted run resumes exactly.

Nothing here performs a significance test.  Focused contrasts are reported as paired
per-participant differences with percentile intervals only.

Usage
-----
    python config_compare_eval.py --run-dir runs/config_compare_2026-07-26 --jobs 4
    python config_compare_eval.py --run-dir runs/config_compare_2026-07-26 --jobs 4 --resume
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import platform
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Sequence

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import win_compat  # noqa: E402  (must precede locked_track_a.core on Windows)

import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
import scipy  # noqa: E402
import sklearn  # noqa: E402

from locked_track_a import core  # noqa: E402

RUNS = HERE / "runs"
TRACK_A_PRIMARY = RUNS / "locked_track_a_2026-07-21" / "primary"
ID_KEYS = ["subject", "trial_id", "block", "bout_index", "y", "label"]


# --------------------------------------------------------------------------------------
# small utilities
# --------------------------------------------------------------------------------------

def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def sha256_json(value: Any) -> str:
    return hashlib.sha256(core.canonical_json(value).encode("utf-8")).hexdigest()


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def json_default(value: Any):
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return float(value)
    if isinstance(value, np.ndarray):
        return value.tolist()
    raise TypeError(f"cannot serialise {type(value)!r}")


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, indent=2, ensure_ascii=False, default=json_default) + "\n",
        encoding="utf-8",
    )


def append_jsonl(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(value, ensure_ascii=False, default=json_default) + "\n")


def write_csv(path: Path, rows: Sequence[dict], fieldnames: Sequence[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(fieldnames))
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


# --------------------------------------------------------------------------------------
# plan and data gates
# --------------------------------------------------------------------------------------

def load_plan(run_dir: Path) -> dict:
    plan_path = run_dir / "run_plan.json"
    sidecar = run_dir / "run_plan.sha256"
    if not plan_path.exists() or not sidecar.exists():
        raise SystemExit(
            "no frozen plan in this run directory; run config_compare_plan.py first"
        )
    recorded = sidecar.read_text(encoding="utf-8").strip()
    actual = sha256_file(plan_path)
    if recorded != actual:
        raise SystemExit(f"run_plan.json has changed since it was frozen: {actual} != {recorded}")
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    if plan.get("plan_schema") != "spinesense_section_7_5_config_compare_plan_v1":
        raise SystemExit("unexpected plan schema")

    for name, expected in plan["code_hashes"].items():
        if name == "config_compare_eval.py":
            continue  # bound below, after the plan is known good
        path = HERE / name
        if expected is None or not path.exists():
            raise SystemExit(f"plan references a missing code file: {name}")
        if sha256_file(path) != expected:
            raise SystemExit(f"{name} changed after the plan was frozen")

    evaluator_sha = sha256_file(Path(__file__).resolve())
    recorded_evaluator = plan["code_hashes"].get("config_compare_eval.py")
    if recorded_evaluator is not None and recorded_evaluator != evaluator_sha:
        raise SystemExit(
            "config_compare_eval.py changed after the plan was frozen; re-freeze the plan "
            "before running (no checkpoints may exist)"
        )
    plan["_evaluator_sha256"] = evaluator_sha
    plan["_plan_sha256"] = actual
    return plan


def load_and_gate_data(plan: dict) -> tuple[pd.DataFrame, np.ndarray, np.ndarray, list[str], dict]:
    gate = plan["data_gate"]
    source_path = HERE / gate["source_table"]
    track_a_path = HERE / gate["track_a_input"]

    source_sha = sha256_file(source_path)
    if source_sha != gate["source_table_sha256"]:
        raise SystemExit(f"channel table drifted from the plan: {source_sha}")
    if sha256_file(track_a_path) != gate["track_a_input_sha256"]:
        raise SystemExit("Track A model matrix drifted from the plan")

    source = pd.read_csv(source_path)
    model_matrix = pd.read_csv(track_a_path)

    if len(source) != len(model_matrix):
        raise SystemExit(f"row counts differ: {len(source)} vs {len(model_matrix)}")

    # row identity
    for key in ("subject", "trial_id", "y", "label"):
        if not np.array_equal(source[key].to_numpy(), model_matrix[key].to_numpy()):
            raise SystemExit(f"row identity gate failed on {key}")
    if source.duplicated(subset=["subject", "trial_id", "block", "bout_index"]).any():
        raise SystemExit("bout composite key is not unique in the channel table")

    # column identity: names and order of the 130 shared predictors
    expected_pairs = [
        f"{channel}__{feature}"
        for channel in gate["expected_channels"]
        for feature in gate["features_per_channel"]
    ]
    actual_pairs = [c for c in model_matrix.columns if c.startswith("pair_")]
    if actual_pairs != expected_pairs:
        raise SystemExit("Track A pair columns or their order differ from the plan")
    if [c for c in source.columns if c in set(expected_pairs)] != expected_pairs:
        raise SystemExit("channel table pair columns or their order differ from the plan")

    # numeric agreement
    difference = float(
        np.max(
            np.abs(
                source[expected_pairs].to_numpy(np.float64)
                - model_matrix[expected_pairs].to_numpy(np.float64)
            )
        )
    )
    if not difference <= float(gate["row_match_atol"]):
        raise SystemExit(f"shared-column agreement {difference} exceeds {gate['row_match_atol']}")

    # single-point channel health
    solo_columns = [
        f"solo_{role}__{feature}"
        for role in ["sacrum", "lower", "mid", "upper", "sternum"]
        for feature in gate["features_per_channel"]
    ]
    missing = [c for c in solo_columns if c not in source.columns]
    if missing:
        raise SystemExit(f"channel table lacks single-point columns: {missing[:5]}")
    solo = source[solo_columns].to_numpy(np.float64)
    if not np.isfinite(solo).all():
        raise SystemExit("single-point channels contain NaN or infinity")
    constant = [c for c in solo_columns if source[c].nunique(dropna=False) <= 1]
    if constant:
        raise SystemExit(f"globally constant single-point columns: {constant[:5]}")

    participants = sorted(source["subject"].unique())
    if participants != list(plan["fold_split"]["participants"]):
        raise SystemExit(f"participant set differs from the plan: {participants}")

    labels = source["label"].map(core.LABEL_TO_INT).to_numpy(int)
    y = source["y"].to_numpy(int)
    if not np.array_equal(labels, y):
        raise SystemExit("label/y mapping differs from the fixed six-class order")

    row_ids = [
        f"{trial}::{block}::{int(bout):04d}"
        for trial, block, bout in zip(source["trial_id"], source["block"], source["bout_index"])
    ]
    if len(set(row_ids)) != len(row_ids):
        raise SystemExit("derived row_id is not unique")

    audit = {
        "n_rows": int(len(source)),
        "n_participants": int(source["subject"].nunique()),
        "shared_columns": len(expected_pairs),
        "shared_column_max_abs_difference": difference,
        "row_match_atol": float(gate["row_match_atol"]),
        "single_point_columns": len(solo_columns),
        "single_point_nan_count": 0,
        "single_point_constant_columns": 0,
        "channel_table_sha256": source_sha,
        "class_support_min": int(
            pd.crosstab(source["subject"], source["label"]).min().min()
        ),
    }
    return source, y, source["subject"].to_numpy(), row_ids, audit


# --------------------------------------------------------------------------------------
# nested LOSO for one (model, configuration)
# --------------------------------------------------------------------------------------

def checkpoint_path(run_dir: Path, model: str, config_key: str, outer: str) -> Path:
    return run_dir / "checkpoints" / f"{model}__{config_key}__{outer}.json"


def _one_outer_fold(
    model: str,
    config: dict,
    grid: list[core.ModelConfig],
    X: np.ndarray,
    y: np.ndarray,
    subjects: np.ndarray,
    row_ids: list[str],
    outer: str,
    participants: list[str],
    seed: int,
    weight_scheme: str,
) -> dict:
    """Full inner selection plus the outer refit for one held-out participant."""

    started = time.perf_counter()
    train_participants = [p for p in participants if p != outer]
    outer_mask = subjects == outer
    train_mask = ~outer_mask

    inner_records: list[dict] = []
    scores: dict[str, list[float]] = {config.config_id: [] for config in grid}
    failed: dict[str, list[str]] = {}

    for inner in train_participants:
        inner_valid = subjects == inner
        inner_train = train_mask & ~inner_valid
        X_train = X[inner_train]
        y_train = y[inner_train]
        subjects_train = subjects[inner_train]
        X_valid = X[inner_valid]
        y_valid = y[inner_valid]
        for grid_config in grid:
            outcome = core.fit_predict_once(
                model,
                grid_config.params,
                seed,
                X_train,
                y_train,
                subjects_train,
                X_valid,
                weight_scheme,
            )
            record = {
                "model": model,
                "config_key": config["config_key"],
                "outer_participant": outer,
                "inner_validation_participant": inner,
                "grid_config_id": grid_config.config_id,
                "simplicity_rank": grid_config.simplicity_rank,
                "params": grid_config.params,
                "status": outcome["status"],
                "n_train_rows": int(y_train.size),
                "n_validation_rows": int(y_valid.size),
                "n_features_after_variance": outcome["n_features_after_variance"],
                "fit_seconds": outcome["fit_seconds"],
                "failure_type": outcome["failure_type"],
                "failure_message": outcome["failure_message"],
                "warnings": outcome["warnings"],
            }
            if outcome["status"] == "failed":
                record["inner_fixed6_macro_f1"] = None
                failed.setdefault(grid_config.config_id, []).append(inner)
            else:
                metrics = core.fixed_six_metrics(y_valid, outcome["prediction"])
                record["inner_fixed6_macro_f1"] = float(metrics["fixed6_macro_f1"])
                scores[grid_config.config_id].append(float(metrics["fixed6_macro_f1"]))
            inner_records.append(record)

    mean_scores: dict[str, float] = {}
    for grid_config in grid:
        values = scores[grid_config.config_id]
        mean_scores[grid_config.config_id] = (
            math.nan if grid_config.config_id in failed or len(values) != len(train_participants)
            else float(np.mean(values))
        )
    selected, selection_details = core.select_config(grid, mean_scores)

    outer_outcome = core.fit_predict_once(
        model,
        selected.params,
        seed,
        X[train_mask],
        y[train_mask],
        subjects[train_mask],
        X[outer_mask],
        weight_scheme,
    )
    if outer_outcome["status"] == "failed":
        raise RuntimeError(
            f"outer refit failed for {model}/{config['config_key']}/{outer}: "
            f"{outer_outcome['failure_type']}: {outer_outcome['failure_message']}"
        )
    prediction = np.asarray(outer_outcome["prediction"], dtype=int)
    y_true = y[outer_mask]
    metrics = core.fixed_six_metrics(y_true, prediction)

    outer_index = np.flatnonzero(outer_mask)
    predictions = [
        {
            "row_id": row_ids[int(index)],
            "source_row_index": int(index),
            "outer_participant": outer,
            "y_true": int(y_true[position]),
            "y_pred": int(prediction[position]),
            "label_true": core.LABEL_NAMES[int(y_true[position])],
            "label_pred": core.LABEL_NAMES[int(prediction[position])],
        }
        for position, index in enumerate(outer_index)
    ]

    return {
        "schema": "config_compare_outer_fold_v1",
        "model": model,
        "config_key": config["config_key"],
        "sensors": config["sensors"],
        "k": config["k"],
        "representation": config["representation"],
        "n_features": config["n_features"],
        "outer_participant": outer,
        "training_participants": train_participants,
        "training_participants_sha256": sha256_json(train_participants),
        "seed": seed,
        "weight_scheme": weight_scheme,
        "selected_config_id": selected.config_id,
        "selected_params": selected.params,
        "selected_simplicity_rank": selected.simplicity_rank,
        "selection_details": selection_details,
        "inner_mean_scores": {k: (None if not np.isfinite(v) else v) for k, v in mean_scores.items()},
        "inner_records": inner_records,
        "ineligible_configs": failed,
        "outer_fixed6_macro_f1": float(metrics["fixed6_macro_f1"]),
        "outer_accuracy": float(metrics["accuracy"]),
        "outer_per_class_recall": [float(v) for v in metrics["recall"]],
        "outer_confusion": metrics["confusion"].tolist(),
        "outer_n_features_after_variance": outer_outcome["n_features_after_variance"],
        "outer_fit_seconds": outer_outcome["fit_seconds"],
        "outer_warnings": outer_outcome["warnings"],
        "predictions": predictions,
        "wall_seconds": float(time.perf_counter() - started),
    }


def evaluate_configuration(task: dict) -> dict:
    """Worker entry point: evaluate one (model, configuration) over all 13 outer folds."""

    win_compat._install_resource_shim()
    run_dir = Path(task["run_dir"])
    model = task["model"]
    config = task["config"]
    grid = [
        core.ModelConfig(
            model=model,
            config_id=item["config_id"],
            simplicity_rank=item["simplicity_rank"],
            params=item["params"],
        )
        for item in task["grid"]
    ]
    X = np.asarray(task["X"], dtype=np.float64)
    y = np.asarray(task["y"], dtype=int)
    subjects = np.asarray(task["subjects"])
    row_ids = list(task["row_ids"])
    participants = list(task["participants"])

    completed = 0
    for outer in participants:
        path = checkpoint_path(run_dir, model, config["config_key"], outer)
        if path.exists():
            try:
                existing = json.loads(path.read_text(encoding="utf-8"))
                if existing.get("schema") == "config_compare_outer_fold_v1":
                    completed += 1
                    continue
            except json.JSONDecodeError:
                path.unlink()
        fold = _one_outer_fold(
            model,
            config,
            grid,
            X,
            y,
            subjects,
            row_ids,
            outer,
            participants,
            task["seed"],
            task["weight_scheme"],
        )
        write_json(path, fold)
        completed += 1
    return {
        "model": model,
        "config_key": config["config_key"],
        "n_features": config["n_features"],
        "folds_present": completed,
    }


def collect_configuration(run_dir: Path, model: str, config_key: str, participants: list[str]) -> list[dict]:
    folds = []
    for outer in participants:
        path = checkpoint_path(run_dir, model, config_key, outer)
        if not path.exists():
            raise SystemExit(f"missing checkpoint: {path.name}")
        folds.append(json.loads(path.read_text(encoding="utf-8")))
    return folds


# --------------------------------------------------------------------------------------
# reproduction: the five-sensor configuration is the first real run
# --------------------------------------------------------------------------------------

def read_track_a_outer(model: str) -> tuple[dict[str, int], dict[str, str]]:
    predictions: dict[str, int] = {}
    selected: dict[str, str] = {}
    with (TRACK_A_PRIMARY / "outer_predictions.csv").open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if row["model"] != model:
                continue
            predictions[row["row_id"]] = int(row["y_pred"])
            selected[row["outer_test_subject"]] = row["selected_config_id"]
    if not predictions:
        raise SystemExit(f"Track A archive has no outer predictions for {model}")
    return predictions, selected


def read_track_a_participants(model: str) -> dict[str, float]:
    values: dict[str, float] = {}
    with (TRACK_A_PRIMARY / "participant_metrics.csv").open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if row["model"] == model:
                values[row["participant"]] = float(row["fixed6_macro_f1"])
    if not values:
        raise SystemExit(f"Track A archive has no participant metrics for {model}")
    return values


def reproduction_check(plan: dict, run_dir: Path, participants: list[str]) -> dict:
    target = plan["reproduction_first_run"]
    config_key = target["configuration"]
    tolerance = float(target["scalar_tolerance"])
    report: dict[str, Any] = {"configuration": config_key, "tolerance": tolerance, "models": {}}
    all_ok = True

    for model in plan["models"]["order"]:
        folds = collect_configuration(run_dir, model, config_key, participants)
        archived_predictions, archived_selected = read_track_a_outer(model)
        archived_participants = read_track_a_participants(model)

        prediction_mismatch = []
        for fold in folds:
            for row in fold["predictions"]:
                archived = archived_predictions.get(row["row_id"])
                if archived is None:
                    prediction_mismatch.append({"row_id": row["row_id"], "reason": "absent in archive"})
                elif archived != row["y_pred"]:
                    prediction_mismatch.append(
                        {"row_id": row["row_id"], "archived": archived, "recomputed": row["y_pred"]}
                    )

        selection_mismatch = [
            {
                "outer_participant": fold["outer_participant"],
                "archived": archived_selected.get(fold["outer_participant"]),
                "recomputed": fold["selected_config_id"],
            }
            for fold in folds
            if archived_selected.get(fold["outer_participant"]) != fold["selected_config_id"]
        ]

        vector = {fold["outer_participant"]: fold["outer_fixed6_macro_f1"] for fold in folds}
        vector_mismatch = [
            {
                "participant": participant,
                "archived": archived_participants.get(participant),
                "recomputed": value,
                "abs_difference": abs(archived_participants.get(participant, math.nan) - value),
            }
            for participant, value in vector.items()
            if not abs(archived_participants.get(participant, math.nan) - value) <= tolerance
        ]

        recomputed_scalar = float(np.mean([vector[p] for p in participants]))
        archived_scalar = float(target["participant_first_macro_f1_mean"][model])
        scalar_ok = abs(recomputed_scalar - archived_scalar) <= tolerance

        model_ok = not prediction_mismatch and not selection_mismatch and not vector_mismatch and scalar_ok
        all_ok = all_ok and model_ok
        report["models"][model] = {
            "n_rows_compared": sum(len(fold["predictions"]) for fold in folds),
            "prediction_mismatches": prediction_mismatch[:20],
            "n_prediction_mismatches": len(prediction_mismatch),
            "selection_mismatches": selection_mismatch,
            "participant_vector_mismatches": vector_mismatch,
            "recomputed_participant_first_macro_f1": recomputed_scalar,
            "archived_participant_first_macro_f1": archived_scalar,
            "scalar_abs_difference": abs(recomputed_scalar - archived_scalar),
            "match": model_ok,
        }

    report["match"] = all_ok
    write_json(run_dir / "reproduction_check.json", report)
    return report


# --------------------------------------------------------------------------------------
# aggregation, contrasts, robustness
# --------------------------------------------------------------------------------------

def summarise(run_dir: Path, plan: dict, participants: list[str]) -> dict:
    results: dict[str, dict[str, Any]] = {}
    for model in plan["models"]["order"]:
        for config in plan["configurations"]:
            key = config["config_key"]
            folds = collect_configuration(run_dir, model, key, participants)
            vector = {fold["outer_participant"]: fold["outer_fixed6_macro_f1"] for fold in folds}
            recall = np.mean([fold["outer_per_class_recall"] for fold in folds], axis=0)
            results.setdefault(key, {"config": config, "models": {}})["models"][model] = {
                "participant_first_macro_f1": float(np.mean([vector[p] for p in participants])),
                "participant_vector": vector,
                "sd": float(np.std([vector[p] for p in participants], ddof=1)),
                "min_participant": float(min(vector.values())),
                "max_participant": float(max(vector.values())),
                "mean_per_class_recall": [float(v) for v in recall],
                "selected_config_ids": {
                    fold["outer_participant"]: fold["selected_config_id"] for fold in folds
                },
            }
    return results


def contrast_table(plan: dict, results: dict, participants: list[str]) -> dict:
    method = plan["interval_method"]
    indices = core.make_bootstrap_indices(
        len(participants),
        repetitions=int(method["repetitions"]),
        seed=int(method["seed"]),
    )
    output: dict[str, Any] = {"method": method, "contrasts": []}
    for contrast in plan["focused_contrasts"]:
        entry = {
            "contrast_id": contrast["contrast_id"],
            "left": contrast["left"],
            "right": contrast["right"],
            "design_reason": contrast["design_reason"],
            "dimension_matched": contrast["dimension_matched"],
            "n_features_left": results[contrast["left"]]["config"]["n_features"],
            "n_features_right": results[contrast["right"]]["config"]["n_features"],
            "models": {},
        }
        for model in plan["models"]["order"]:
            left = results[contrast["left"]]["models"][model]["participant_vector"]
            right = results[contrast["right"]]["models"][model]["participant_vector"]
            differences = np.array([left[p] - right[p] for p in participants], dtype=np.float64)
            low, high = core.bootstrap_mean_ci(differences, indices, alpha=float(method["alpha"]))
            entry["models"][model] = {
                "per_participant_difference": {p: float(d) for p, d in zip(participants, differences)},
                "mean_difference": float(np.mean(differences)),
                "median_difference": float(np.median(differences)),
                "ci_low": float(low),
                "ci_high": float(high),
                "n_participants_favouring_left": int(np.sum(differences > 0)),
                "n_participants_favouring_right": int(np.sum(differences < 0)),
                "n_participants_tied": int(np.sum(differences == 0)),
                "significance_testing": "none",
            }
        entry["sign_agreement_between_models"] = bool(
            np.sign(entry["models"][plan["models"]["primary"]]["mean_difference"])
            == np.sign(entry["models"][plan["models"]["robustness"]]["mean_difference"])
        )
        output["contrasts"].append(entry)
    return output


def robustness_report(plan: dict, results: dict, contrasts: dict) -> dict:
    primary = plan["models"]["primary"]
    robust = plan["models"]["robustness"]
    keys = [config["config_key"] for config in plan["configurations"]]
    a = np.array([results[k]["models"][primary]["participant_first_macro_f1"] for k in keys])
    b = np.array([results[k]["models"][robust]["participant_first_macro_f1"] for k in keys])

    def rank(values: np.ndarray) -> np.ndarray:
        order = np.argsort(values, kind="mergesort")
        ranks = np.empty(values.size, dtype=np.float64)
        ranks[order] = np.arange(1, values.size + 1, dtype=np.float64)
        # average ties
        for value in np.unique(values):
            mask = values == value
            if mask.sum() > 1:
                ranks[mask] = ranks[mask].mean()
        return ranks

    ra, rb = rank(a), rank(b)
    rho = float(np.corrcoef(ra, rb)[0, 1])
    return {
        "method": plan["model_family_robustness"]["method"],
        "primary_model": primary,
        "robustness_model": robust,
        "n_configurations": len(keys),
        "spearman_rho": rho,
        "per_contrast_sign_agreement": {
            entry["contrast_id"]: entry["sign_agreement_between_models"]
            for entry in contrasts["contrasts"]
        },
        "all_contrasts_agree_in_sign": all(
            entry["sign_agreement_between_models"] for entry in contrasts["contrasts"]
        ),
        "reported": plan["model_family_robustness"]["reported"],
        "not_reported": plan["model_family_robustness"]["not_reported"],
        "note": (
            "Rank agreement describes whether the ordering of configurations survives a change "
            "of model family. No correlation test and no p-value is computed."
        ),
    }


def detailed_confusion(run_dir: Path, plan: dict, participants: list[str]) -> None:
    for config_key in plan["detailed_reporting"]["configurations"]:
        for model in plan["models"]["order"]:
            folds = collect_configuration(run_dir, model, config_key, participants)
            true_all: list[int] = []
            pred_all: list[int] = []
            who: list[str] = []
            for fold in folds:
                for row in fold["predictions"]:
                    true_all.append(row["y_true"])
                    pred_all.append(row["y_pred"])
                    who.append(row["outer_participant"])
            normalised, per_participant = core.participant_normalized_confusion(true_all, pred_all, who)
            metrics = core.fixed_six_metrics(true_all, pred_all)
            write_json(
                run_dir / "confusion" / f"{config_key}__{model}.json",
                {
                    "config_key": config_key,
                    "model": model,
                    "label_order": core.LABEL_NAMES,
                    "participant_normalised_confusion": normalised.tolist(),
                    "per_participant_confusion": {k: v.tolist() for k, v in per_participant.items()},
                    "per_class_recall_pooled": [float(v) for v in metrics["recall"]],
                    "per_class_precision_pooled": [float(v) for v in metrics["precision"]],
                    "note": (
                        "Reported for this configuration because it enters one of the four "
                        "focused contrasts."
                    ),
                },
            )


# --------------------------------------------------------------------------------------
# orchestration
# --------------------------------------------------------------------------------------

def environment_lock() -> dict:
    return {
        "captured_utc": utc_now(),
        "python_version": platform.python_version(),
        "python_executable": sys.executable,
        "platform": platform.platform(),
        "machine": platform.machine(),
        "processor": platform.processor(),
        "numpy": np.__version__,
        "pandas": pd.__version__,
        "scikit_learn": sklearn.__version__,
        "scipy": scipy.__version__,
        "windows_compatibility": win_compat.describe(),
    }


def build_tasks(plan: dict, X_all: pd.DataFrame, y: np.ndarray, subjects: np.ndarray,
                row_ids: list[str], run_dir: Path, only: Iterable[str] | None = None) -> list[dict]:
    participants = list(plan["fold_split"]["participants"])
    wanted = set(only) if only is not None else None
    tasks: list[dict] = []
    for model in plan["models"]["order"]:
        grid = plan["models"]["grids"][model]
        for config in plan["configurations"]:
            if wanted is not None and config["config_key"] not in wanted:
                continue
            if all(
                checkpoint_path(run_dir, model, config["config_key"], outer).exists()
                for outer in participants
            ):
                continue
            tasks.append(
                {
                    "run_dir": str(run_dir),
                    "model": model,
                    "config": config,
                    "grid": grid,
                    "X": X_all[config["feature_columns"]].to_numpy(np.float64),
                    "y": y,
                    "subjects": subjects,
                    "row_ids": row_ids,
                    "participants": participants,
                    "seed": int(plan["inherited_from_locked_track_a"]["seed"]),
                    "weight_scheme": plan["inherited_from_locked_track_a"]["weight_scheme"],
                }
            )
    # longest first: cost grows with grid size times feature count
    tasks.sort(key=lambda t: len(t["grid"]) * t["config"]["n_features"], reverse=True)
    return tasks


def run_tasks(tasks: list[dict], jobs: int, run_dir: Path, stage: str) -> None:
    if not tasks:
        print(f"[{stage}] nothing to do; every checkpoint is present")
        return
    print(f"[{stage}] {len(tasks)} configuration-model tasks on {jobs} worker(s)")
    started = time.perf_counter()
    if jobs <= 1:
        for index, task in enumerate(tasks, 1):
            outcome = evaluate_configuration(task)
            print(
                f"[{stage}] {index}/{len(tasks)} {outcome['model']:9s} "
                f"{outcome['config_key']:34s} d={outcome['n_features']:3d} done",
                flush=True,
            )
            append_jsonl(run_dir / "ledgers" / "runtime.jsonl",
                         {"stage": stage, "utc": utc_now(), **outcome})
    else:
        with ProcessPoolExecutor(max_workers=jobs) as pool:
            futures = {pool.submit(evaluate_configuration, task): task for task in tasks}
            for index, future in enumerate(as_completed(futures), 1):
                task = futures[future]
                outcome = future.result()
                print(
                    f"[{stage}] {index}/{len(tasks)} {outcome['model']:9s} "
                    f"{outcome['config_key']:34s} d={outcome['n_features']:3d} done",
                    flush=True,
                )
                append_jsonl(run_dir / "ledgers" / "runtime.jsonl",
                             {"stage": stage, "utc": utc_now(), **outcome})
    print(f"[{stage}] wall {(time.perf_counter() - started) / 60:.1f} min")


def export_tables(run_dir: Path, plan: dict, results: dict, participants: list[str]) -> None:
    summary_rows = []
    for config in plan["configurations"]:
        key = config["config_key"]
        row = {
            "config_key": key,
            "k": config["k"],
            "representation": config["representation"],
            "n_features": config["n_features"],
            "sensors": "+".join(config["sensors"]),
        }
        for model in plan["models"]["order"]:
            entry = results[key]["models"][model]
            row[f"{model}__participant_first_macro_f1"] = entry["participant_first_macro_f1"]
            row[f"{model}__sd"] = entry["sd"]
            row[f"{model}__min_participant"] = entry["min_participant"]
        summary_rows.append(row)
    summary_rows.sort(key=lambda r: (-r["k"], r["config_key"]))
    write_csv(run_dir / "config_summary.csv", summary_rows, list(summary_rows[0]))

    participant_rows = []
    selection_rows = []
    prediction_rows = []
    inner_rows = []
    for model in plan["models"]["order"]:
        for config in plan["configurations"]:
            key = config["config_key"]
            for fold in collect_configuration(run_dir, model, key, participants):
                participant_rows.append(
                    {
                        "model": model,
                        "config_key": key,
                        "n_features": config["n_features"],
                        "participant": fold["outer_participant"],
                        "n_bouts": len(fold["predictions"]),
                        "fixed6_macro_f1": fold["outer_fixed6_macro_f1"],
                        "accuracy": fold["outer_accuracy"],
                    }
                )
                selection_rows.append(
                    {
                        "model": model,
                        "config_key": key,
                        "outer_participant": fold["outer_participant"],
                        "selected_config_id": fold["selected_config_id"],
                        "selected_simplicity_rank": fold["selected_simplicity_rank"],
                        "selected_params_json": core.canonical_json(fold["selected_params"]),
                        "best_mean_inner_score": fold["selection_details"]["best_mean_score"],
                        "selected_mean_inner_score": fold["selection_details"]["selected_mean_score"],
                        "n_practically_tied": len(
                            fold["selection_details"]["practically_tied_config_ids"]
                        ),
                    }
                )
                for row in fold["predictions"]:
                    prediction_rows.append({"model": model, "config_key": key, **row})
                for record in fold["inner_records"]:
                    inner_rows.append(
                        {
                            "model": record["model"],
                            "config_key": record["config_key"],
                            "outer_participant": record["outer_participant"],
                            "inner_validation_participant": record["inner_validation_participant"],
                            "grid_config_id": record["grid_config_id"],
                            "simplicity_rank": record["simplicity_rank"],
                            "status": record["status"],
                            "inner_fixed6_macro_f1": record["inner_fixed6_macro_f1"],
                            "n_features_after_variance": record["n_features_after_variance"],
                            "fit_seconds": record["fit_seconds"],
                        }
                    )
                    if record["status"] != "ok":
                        append_jsonl(
                            run_dir / "ledgers" / "warnings_failures.jsonl",
                            {"scope": "inner", "utc": utc_now(), **record},
                        )

    write_csv(run_dir / "participant_metrics.csv", participant_rows, list(participant_rows[0]))
    write_csv(run_dir / "selected_configs.csv", selection_rows, list(selection_rows[0]))
    write_csv(run_dir / "outer_predictions.csv", prediction_rows, list(prediction_rows[0]))
    write_csv(run_dir / "inner_scores.csv", inner_rows, list(inner_rows[0]))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Section 7.5 sensor-configuration comparison.")
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--jobs", type=int, default=1)
    parser.add_argument("--resume", action="store_true",
                        help="Continue from existing checkpoints (the default behaviour anyway).")
    parser.add_argument("--stop-after-reproduction", action="store_true",
                        help="Run only the five-sensor configuration and its comparison.")
    args = parser.parse_args(argv)

    run_dir = args.run_dir.resolve()
    plan = load_plan(run_dir)
    participants = list(plan["fold_split"]["participants"])
    print(f"plan sha256   : {plan['_plan_sha256']}")
    print(f"analysis id   : {plan['analysis_id']}")
    print(f"supersedes    : {plan['supersedes']['document']}")

    source, y, subjects, row_ids, audit = load_and_gate_data(plan)
    write_json(run_dir / "data_gate.json", audit)
    write_json(run_dir / "environment_lock.json", environment_lock())
    print(
        f"data gate     : {audit['n_rows']} rows, {audit['n_participants']} participants, "
        f"shared-column max abs diff {audit['shared_column_max_abs_difference']}"
    )

    # ---- stage 1: the five-sensor configuration is the first real run ----
    reproduction_key = plan["reproduction_first_run"]["configuration"]
    stage1 = build_tasks(plan, source, y, subjects, row_ids, run_dir, only=[reproduction_key])
    run_tasks(stage1, min(args.jobs, 2), run_dir, "reproduction")
    report = reproduction_check(plan, run_dir, participants)
    for model, detail in report["models"].items():
        print(
            f"reproduction  : {model:9s} rows={detail['n_rows_compared']} "
            f"pred_mismatch={detail['n_prediction_mismatches']} "
            f"fold_mismatch={len(detail['selection_mismatches'])} "
            f"scalar_delta={detail['scalar_abs_difference']:.3e} "
            f"{'OK' if detail['match'] else 'MISMATCH'}"
        )
    if not report["match"]:
        raise SystemExit(
            "reproduction of the five-sensor configuration does not match the Track A archive; "
            "refusing to evaluate the remaining configurations (see reproduction_check.json)"
        )
    if args.stop_after_reproduction:
        print("stopping after the reproduction stage as requested")
        return 0

    # ---- stage 2: the remaining 30 configurations ----
    stage2 = build_tasks(plan, source, y, subjects, row_ids, run_dir)
    run_tasks(stage2, args.jobs, run_dir, "configurations")

    # ---- aggregation ----
    results = summarise(run_dir, plan, participants)
    contrasts = contrast_table(plan, results, participants)
    robustness = robustness_report(plan, results, contrasts)
    detailed_confusion(run_dir, plan, participants)
    export_tables(run_dir, plan, results, participants)

    write_json(run_dir / "config_results.json", {
        "analysis_id": plan["analysis_id"],
        "plan_sha256": plan["_plan_sha256"],
        "configurations": results,
    })
    write_json(run_dir / "focused_contrasts.json", contrasts)
    write_json(run_dir / "model_family_robustness.json", robustness)

    manifest = {
        "analysis_id": plan["analysis_id"],
        "completed_utc": utc_now(),
        "manuscript_section": plan["manuscript_section"],
        "supersedes": plan["supersedes"],
        "plan_sha256": plan["_plan_sha256"],
        "evaluator_sha256": plan["_evaluator_sha256"],
        "code_hashes": plan["code_hashes"],
        "input_hashes": {
            plan["data_gate"]["source_table"]: plan["data_gate"]["source_table_sha256"],
            plan["data_gate"]["track_a_input"]: plan["data_gate"]["track_a_input_sha256"],
        },
        "data_gate": audit,
        "environment": environment_lock(),
        "reproduction_check": report,
        "n_configurations": plan["n_configurations"],
        "models": plan["models"]["order"],
        "interval_method": plan["interval_method"],
        "significance_testing": "none",
        "evidence_boundary": plan["evidence_boundary"],
        "output_hashes": {
            name: sha256_file(run_dir / name)
            for name in [
                "config_summary.csv",
                "config_results.json",
                "focused_contrasts.json",
                "model_family_robustness.json",
                "participant_metrics.csv",
                "selected_configs.csv",
                "outer_predictions.csv",
                "inner_scores.csv",
                "reproduction_check.json",
            ]
            if (run_dir / name).exists()
        },
    }
    write_json(run_dir / "run_manifest.json", manifest)

    print("\n--- 31 configurations, participant-equal-weight macro F1 ---")
    primary = plan["models"]["primary"]
    ordered = sorted(
        plan["configurations"],
        key=lambda c: results[c["config_key"]]["models"][primary]["participant_first_macro_f1"],
        reverse=True,
    )
    for config in ordered:
        key = config["config_key"]
        line = f"  {key:34s} k={config['k']} d={config['n_features']:3d}"
        for model in plan["models"]["order"]:
            line += f"  {model}={results[key]['models'][model]['participant_first_macro_f1']:.4f}"
        print(line)

    print("\n--- focused contrasts (differences and 95% percentile intervals, no tests) ---")
    for entry in contrasts["contrasts"]:
        detail = entry["models"][primary]
        print(
            f"  {entry['contrast_id']}  {entry['left']} - {entry['right']}"
            f"  mean={detail['mean_difference']:+.4f}"
            f"  CI=[{detail['ci_low']:+.4f}, {detail['ci_high']:+.4f}]"
            f"  dim_matched={entry['dimension_matched']}"
        )
    print(
        f"\nmodel-family rank agreement: Spearman rho = {robustness['spearman_rho']:.4f}; "
        f"all contrast signs agree = {robustness['all_contrasts_agree_in_sign']}"
    )
    print(f"\nwrote {run_dir / 'run_manifest.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
