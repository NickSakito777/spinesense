#!/usr/bin/env python3
"""Archive the post-hoc time/order-only negative control for locked Track A.

The evaluative baseline uses one scalar only: each bout's zero-based acquisition rank
within its participant, divided by (n_participant - 1).  For every outer participant,
the classifier computes six pooled class centroids from the other 12 participants and
assigns each held-out bout to the nearest centroid.  No IMU value is used.

This is a deliberately post-hoc protocol-confounding diagnostic.  It is stored outside
the locked Track A run and must not be interpreted as a decomposition of IMU performance.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import subprocess
import sys
import time
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

import numpy as np
import pandas as pd

from locked_track_a.core import LABEL_NAMES, LABELS, fixed_six_metrics


HERE = Path(__file__).resolve().parent
LOCKED_RUN = HERE / "runs" / "locked_track_a_2026-07-21"
DEFAULT_OUTPUT = (
    HERE
    / "runs"
    / "posthoc_diagnostics_2026-07-22"
    / "time_order_only_baseline"
)
ROW_REGISTRY = LOCKED_RUN / "p0" / "row_registry.csv"
PRIMARY_PREDICTIONS = LOCKED_RUN / "primary" / "outer_predictions.csv"
BLOCK_TO_LABEL = {
    "B1": 0,
    "B2": 1,
    "B3": 2,
    "B4": 3,
    "B5": 4,
    "B6": 5,
    "B6a": 5,
    "B6b": 5,
}


def canonical_json(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def package_version(name: str) -> str:
    try:
        return metadata.version(name)
    except metadata.PackageNotFoundError:
        return "not-installed"


def git_snapshot() -> dict[str, Any]:
    try:
        head = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=HERE,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        status = subprocess.run(
            ["git", "status", "--short"],
            cwd=HERE,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.splitlines()
        return {"head": head, "status_short": status}
    except (OSError, subprocess.CalledProcessError) as error:
        return {"head": None, "status_short": [], "capture_error": str(error)}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args()


def class_metric_rows(
    participant: str,
    metrics: dict[str, Any],
) -> list[dict[str, Any]]:
    return [
        {
            "participant": participant,
            "class_id": int(class_id),
            "label": LABEL_NAMES[int(class_id)],
            "precision": float(metrics["precision"][class_id]),
            "recall": float(metrics["recall"][class_id]),
            "f1": float(metrics["f1"][class_id]),
            "support": int(metrics["support"][class_id]),
        }
        for class_id in LABELS
    ]


def main() -> int:
    args = parse_args()
    started = time.perf_counter()
    output_dir = args.output_dir.resolve()
    if output_dir.exists() and any(output_dir.iterdir()):
        raise FileExistsError(f"refusing to overwrite non-empty output directory: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)

    registry = pd.read_csv(ROW_REGISTRY).sort_values("source_row_index").reset_index(drop=True)
    required = {
        "row_id",
        "source_row_index",
        "subject",
        "block",
        "bout_index",
        "bout_start_s",
        "y",
        "label",
        "input_csv_sha256",
    }
    if not required.issubset(registry.columns):
        raise AssertionError(f"row registry missing columns: {sorted(required - set(registry.columns))}")
    if len(registry) != 1387 or registry["subject"].nunique() != 13:
        raise AssertionError("unexpected frozen cohort dimensions")
    if not np.array_equal(registry["source_row_index"].to_numpy(), np.arange(len(registry))):
        raise AssertionError("source_row_index is not a complete unique row sequence")
    if registry["row_id"].duplicated().any():
        raise AssertionError("row_id is not unique")
    if registry["input_csv_sha256"].nunique() != 1:
        raise AssertionError("row registry references more than one frozen input hash")
    if set(registry["block"]) - set(BLOCK_TO_LABEL):
        raise AssertionError("unknown protocol block")
    expected_from_block = registry["block"].map(BLOCK_TO_LABEL).to_numpy(dtype=int)
    y_true = registry["y"].to_numpy(dtype=int)
    if not np.array_equal(expected_from_block, y_true):
        raise AssertionError("protocol block no longer maps deterministically to label")

    subjects = registry["subject"].astype(str).to_numpy()
    within_rank = registry.groupby("subject", sort=False).cumcount().to_numpy(dtype=float)
    participant_n = (
        registry.groupby("subject", sort=False)["subject"].transform("size").to_numpy(dtype=float)
    )
    if np.any(participant_n <= 1):
        raise AssertionError("cannot normalize acquisition rank for a singleton participant")
    normalized_rank = within_rank / (participant_n - 1.0)
    registry["within_participant_acquisition_rank"] = within_rank.astype(int)
    registry["normalized_acquisition_rank"] = normalized_rank

    for participant, subset in registry.groupby("subject", sort=True):
        if not subset["source_row_index"].is_monotonic_increasing:
            raise AssertionError(f"source rows are not monotonic for {participant}")
        if not subset["y"].is_monotonic_increasing:
            raise AssertionError(f"labels are not in fixed protocol order for {participant}")

    prediction = np.full(len(registry), -1, dtype=int)
    nearest_distance = np.full(len(registry), np.nan, dtype=float)
    center_rows: list[dict[str, Any]] = []
    participant_rows: list[dict[str, Any]] = []
    participant_class_rows: list[dict[str, Any]] = []
    confusion_rows: list[dict[str, Any]] = []

    primary = pd.read_csv(PRIMARY_PREDICTIONS)
    primary = primary.loc[primary["model"] == "logistic"].sort_values("source_row_index")
    if not np.array_equal(primary["source_row_index"].to_numpy(), np.arange(len(registry))):
        raise AssertionError("primary Logistic predictions do not cover every frozen row exactly once")
    if not np.array_equal(primary["y_true"].to_numpy(dtype=int), y_true):
        raise AssertionError("primary Logistic labels do not align with row registry")
    primary_prediction = primary["y_pred"].to_numpy(dtype=int)

    participants = sorted(np.unique(subjects))
    for outer_subject in participants:
        test_mask = subjects == outer_subject
        train_mask = ~test_mask
        train_participants = sorted(np.unique(subjects[train_mask]))
        test_participants = sorted(np.unique(subjects[test_mask]))
        overlap = sorted(set(train_participants) & set(test_participants))
        if overlap or test_participants != [outer_subject]:
            raise AssertionError("outer participant isolation failed")

        centers = np.asarray(
            [normalized_rank[train_mask & (y_true == class_id)].mean() for class_id in LABELS],
            dtype=float,
        )
        if not np.all(np.diff(centers) > 0):
            raise AssertionError("training class order centroids are not strictly increasing")
        distances = np.abs(normalized_rank[test_mask, None] - centers[None, :])
        fold_prediction = np.argmin(distances, axis=1).astype(int)
        prediction[test_mask] = fold_prediction
        nearest_distance[test_mask] = distances[np.arange(test_mask.sum()), fold_prediction]

        for class_id, center in zip(LABELS, centers):
            center_rows.append(
                {
                    "outer_test_subject": outer_subject,
                    "class_id": int(class_id),
                    "label": LABEL_NAMES[int(class_id)],
                    "training_pooled_normalized_rank_centroid": float(center),
                    "n_training_bouts": int(np.sum(train_mask & (y_true == class_id))),
                }
            )

        metrics = fixed_six_metrics(y_true[test_mask], fold_prediction)
        primary_metrics = fixed_six_metrics(y_true[test_mask], primary_prediction[test_mask])
        participant_rows.append(
            {
                "participant": outer_subject,
                "n_test_bouts": int(test_mask.sum()),
                "train_test_participant_overlap": canonical_json(overlap),
                "order_only_fixed6_macro_f1": metrics["fixed6_macro_f1"],
                "order_only_fixed6_balanced_accuracy": metrics["fixed6_balanced_accuracy"],
                "order_only_accuracy": metrics["accuracy"],
                "primary_logistic_fixed6_macro_f1": primary_metrics["fixed6_macro_f1"],
                "primary_logistic_accuracy": primary_metrics["accuracy"],
                "primary_minus_order_macro_f1": (
                    primary_metrics["fixed6_macro_f1"] - metrics["fixed6_macro_f1"]
                ),
                "primary_minus_order_accuracy": primary_metrics["accuracy"] - metrics["accuracy"],
            }
        )
        participant_class_rows.extend(class_metric_rows(outer_subject, metrics))
        cm = metrics["confusion"].astype(float)
        row_sums = cm.sum(axis=1, keepdims=True)
        if np.any(row_sums == 0):
            raise AssertionError(f"{outer_subject} is missing a fixed class")
        normalized_cm = cm / row_sums
        for true_class in LABELS:
            for predicted_class in LABELS:
                confusion_rows.append(
                    {
                        "participant": outer_subject,
                        "true_class_id": int(true_class),
                        "true_label": LABEL_NAMES[int(true_class)],
                        "predicted_class_id": int(predicted_class),
                        "predicted_label": LABEL_NAMES[int(predicted_class)],
                        "count": int(cm[true_class, predicted_class]),
                        "true_row_normalized": float(normalized_cm[true_class, predicted_class]),
                    }
                )

    if np.any(prediction < 0) or not np.isfinite(nearest_distance).all():
        raise AssertionError("not every bout received exactly one order-only outer prediction")

    participant_metrics = pd.DataFrame(participant_rows).sort_values("participant")
    participant_class = pd.DataFrame(participant_class_rows)
    confusion = pd.DataFrame(confusion_rows)
    centers = pd.DataFrame(center_rows)
    outer_predictions = registry[
        [
            "row_id",
            "source_row_index",
            "subject",
            "block",
            "bout_index",
            "bout_start_s",
            "y",
            "label",
            "within_participant_acquisition_rank",
            "normalized_acquisition_rank",
        ]
    ].copy()
    outer_predictions = outer_predictions.rename(columns={"subject": "outer_test_subject"})
    outer_predictions["y_pred"] = prediction
    outer_predictions["label_pred"] = [LABEL_NAMES[item] for item in prediction]
    outer_predictions["nearest_centroid_distance"] = nearest_distance

    overall_centers = pd.DataFrame(
        [
            {
                "class_id": int(class_id),
                "label": LABEL_NAMES[int(class_id)],
                "all_data_pooled_normalized_rank_centroid": float(
                    normalized_rank[y_true == class_id].mean()
                ),
                "n_bouts": int(np.sum(y_true == class_id)),
            }
            for class_id in LABELS
        ]
    )

    # Demonstrate why a naive first-80%/last-20% split is invalid: time and class are tied.
    early_train = np.zeros(len(registry), dtype=bool)
    for participant in participants:
        indices = np.flatnonzero(subjects == participant)
        split = int(np.floor(0.8 * len(indices)))
        early_train[indices[:split]] = True
    naive_support_rows: list[dict[str, Any]] = []
    for participant in [*participants, "ALL"]:
        participant_mask = np.ones(len(registry), dtype=bool) if participant == "ALL" else subjects == participant
        for class_id in LABELS:
            class_mask = y_true == class_id
            naive_support_rows.append(
                {
                    "participant": participant,
                    "class_id": int(class_id),
                    "label": LABEL_NAMES[int(class_id)],
                    "early_80_train_support": int(np.sum(participant_mask & class_mask & early_train)),
                    "late_20_test_support": int(np.sum(participant_mask & class_mask & ~early_train)),
                }
            )
    naive_support = pd.DataFrame(naive_support_rows)

    oracle_prediction = expected_from_block
    oracle_metrics = fixed_six_metrics(y_true, oracle_prediction)
    pooled_metrics = fixed_six_metrics(y_true, prediction)
    mean_confusion = (
        confusion.groupby(
            ["true_class_id", "true_label", "predicted_class_id", "predicted_label"],
            as_index=False,
        )["true_row_normalized"]
        .mean()
        .rename(columns={"true_row_normalized": "participant_mean_true_row_normalized"})
    )
    class_summary = (
        participant_class.groupby(["class_id", "label"], as_index=False)
        .agg(
            participant_mean_precision=("precision", "mean"),
            participant_mean_recall=("recall", "mean"),
            participant_mean_f1=("f1", "mean"),
            participant_sd_f1=("f1", "std"),
            total_support=("support", "sum"),
        )
        .sort_values("class_id")
    )

    summary = {
        "analysis_id": "spinesense_track_a_posthoc_time_order_only_baseline_2026-07-22",
        "status": "post-hoc negative control; separate from locked Track A",
        "n_participants": len(participants),
        "n_bouts": len(registry),
        "predictors": ["normalized_acquisition_rank"],
        "uses_imu": False,
        "validation": "13-fold participant LOSO",
        "classifier": "nearest pooled training-class centroid in one-dimensional normalized rank",
        "transductive_order_feature": True,
        "transductive_limitation": (
            "Rank normalization uses the held-out participant's complete-session bout count. "
            "It uses no held-out label or IMU value, but it is not an online/deployable predictor."
        ),
        "participant_first_fixed6_macro_f1_mean": float(
            participant_metrics["order_only_fixed6_macro_f1"].mean()
        ),
        "participant_first_fixed6_macro_f1_sd": float(
            participant_metrics["order_only_fixed6_macro_f1"].std(ddof=1)
        ),
        "participant_first_accuracy_mean": float(
            participant_metrics["order_only_accuracy"].mean()
        ),
        "participant_first_accuracy_sd": float(
            participant_metrics["order_only_accuracy"].std(ddof=1)
        ),
        "pooled_fixed6_macro_f1_descriptive": pooled_metrics["fixed6_macro_f1"],
        "pooled_accuracy_descriptive": pooled_metrics["accuracy"],
        "primary_logistic_participant_first_macro_f1_mean": float(
            participant_metrics["primary_logistic_fixed6_macro_f1"].mean()
        ),
        "primary_logistic_participant_first_accuracy_mean": float(
            participant_metrics["primary_logistic_accuracy"].mean()
        ),
        "participants_primary_logistic_higher_than_order_only": int(
            np.sum(
                participant_metrics["primary_logistic_fixed6_macro_f1"]
                > participant_metrics["order_only_fixed6_macro_f1"]
            )
        ),
        "protocol_block_id_oracle_fixed6_macro_f1": oracle_metrics["fixed6_macro_f1"],
        "protocol_block_id_oracle_accuracy": oracle_metrics["accuracy"],
        "naive_early80_late20_split_evaluative": False,
        "naive_early80_late20_reason": (
            "Fixed protocol order confounds time with label; the late 20% has no flexion, "
            "extension, or left-bend bouts and is dominated by right twist."
        ),
        "interpretation_boundary": (
            "The baseline shows that fixed acquisition order alone carries substantial label "
            "information. It does not quantify a causal percentage of the IMU score, and the "
            "difference from Logistic must not be labelled pure IMU contribution."
        ),
        "runtime_seconds": float(time.perf_counter() - started),
        "warnings": [],
        "failures": [],
    }

    outer_predictions.to_csv(output_dir / "outer_predictions.csv", index=False)
    participant_metrics.to_csv(output_dir / "participant_metrics.csv", index=False)
    participant_class.to_csv(output_dir / "participant_class_metrics.csv", index=False)
    class_summary.to_csv(output_dir / "participant_first_class_summary.csv", index=False)
    confusion.to_csv(output_dir / "participant_confusion_long.csv", index=False)
    mean_confusion.to_csv(output_dir / "participant_normalized_confusion.csv", index=False)
    centers.to_csv(output_dir / "outer_training_class_centroids.csv", index=False)
    overall_centers.to_csv(output_dir / "overall_class_centroids_descriptive.csv", index=False)
    naive_support.to_csv(output_dir / "naive_chronological_split_support.csv", index=False)
    pd.DataFrame(
        columns=["stage", "outer_subject", "category", "message"]
    ).to_csv(output_dir / "warnings_failures.csv", index=False)
    write_json(output_dir / "summary.json", summary)

    environment = {
        "python_version": sys.version,
        "python_executable": sys.executable,
        "platform": platform.platform(),
        "packages": {
            name: package_version(name)
            for name in ("numpy", "pandas", "scipy", "scikit-learn")
        },
        "thread_environment": {
            name: os.environ.get(name)
            for name in (
                "OMP_NUM_THREADS",
                "OPENBLAS_NUM_THREADS",
                "MKL_NUM_THREADS",
                "VECLIB_MAXIMUM_THREADS",
                "NUMEXPR_NUM_THREADS",
            )
        },
    }
    write_json(output_dir / "environment_lock.json", environment)

    manifest = {
        "analysis_id": summary["analysis_id"],
        "run_status": "complete",
        "analysis_status": "exploratory post-hoc negative control",
        "locked_primary_modified": False,
        "design": {
            "source_order": "frozen source_row_index within participant",
            "order_transform": "zero_based_rank / (participant_bout_count - 1)",
            "predictors": ["normalized_acquisition_rank"],
            "uses_imu": False,
            "outer_validation": "13-fold participant LOSO",
            "training_rule": "pooled mean normalized rank for each of six classes",
            "prediction_rule": "nearest class centroid; lowest class id resolves exact ties",
            "held_out_session_information": (
                "normalization uses the held-out participant's total bout count n_s, "
                "but no held-out label or IMU value"
            ),
            "hyperparameters": [],
            "tuning": "none",
            "metric": "participant-first fixed-six-class macro-F1",
        },
        "sources": {
            "row_registry": {
                "path": str(ROW_REGISTRY.resolve()),
                "sha256": sha256_file(ROW_REGISTRY),
            },
            "frozen_input_csv_sha256": str(registry["input_csv_sha256"].iloc[0]),
            "primary_outer_predictions": {
                "path": str(PRIMARY_PREDICTIONS.resolve()),
                "sha256": sha256_file(PRIMARY_PREDICTIONS),
            },
            "script": {
                "path": str(Path(__file__).resolve()),
                "sha256": sha256_file(Path(__file__).resolve()),
            },
        },
        "git": git_snapshot(),
        "environment_file": "environment_lock.json",
        "summary_file": "summary.json",
        "warnings_failures_file": "warnings_failures.csv",
        "summary": summary,
    }
    write_json(output_dir / "manifest.json", manifest)

    artifact_rows = []
    for path in sorted(item for item in output_dir.iterdir() if item.is_file()):
        if path.name == "artifact_manifest.csv":
            continue
        artifact_rows.append(
            {"relative_path": path.name, "size_bytes": path.stat().st_size, "sha256": sha256_file(path)}
        )
    pd.DataFrame(artifact_rows).to_csv(output_dir / "artifact_manifest.csv", index=False)
    print(canonical_json(summary), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
