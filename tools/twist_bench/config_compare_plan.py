"""Freeze the read-only run plan for the Section 7.5 sensor-configuration comparison.

This script computes NO scores.  It enumerates every decision that Section 7.5 commits to,
binds every input file by hash, and writes a read-only ``run_plan.json`` plus a sidecar
``run_plan.sha256``.  ``config_compare_eval.py`` refuses to run without a matching pair, so
the configuration list, column order, fold split, parameter grids, focused contrasts,
interval algorithm, and seeds are all fixed before the first fit.

Section 7.5 supersedes the sensor-ablation paragraph of the V3 blueprint.  The blueprint
still describes fixed-root channels, logistic + random forest, pooled metrics, and a ten
contrast Holm family.  None of those apply here: this plan uses all-pairs channels for
k >= 2, logistic as the primary procedure with RBF-SVM as a model-family check,
participant-equal-weight macro F1, and four focused contrasts with intervals only.

Usage
-----
    python config_compare_plan.py --out-dir runs/config_compare_2026-07-26
"""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import os
import stat
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import win_compat  # noqa: F401,E402  (installs the POSIX ``resource`` shim on Windows)

from locked_track_a import core  # noqa: E402

RUNS = HERE / "runs"
SOURCE_DIR = RUNS / "ablation_2026-07-20"
SOURCE_TABLE = SOURCE_DIR / "channel_features.csv"
SOURCE_MANIFEST = SOURCE_DIR / "ablation_manifest.json"
TRACK_A_DIR = RUNS / "locked_track_a_2026-07-21"
TRACK_A_INPUT = RUNS / "allpairs_2026-07-21" / "features_model.csv"
TRACK_A_DATASET_MANIFEST = RUNS / "allpairs_2026-07-21" / "dataset_manifest.json"
TRACK_A_MODEL_SUMMARY = TRACK_A_DIR / "primary" / "model_summary.csv"
TRACK_A_OUTER_PREDICTIONS = TRACK_A_DIR / "primary" / "outer_predictions.csv"
TRACK_A_PARTICIPANT_METRICS = TRACK_A_DIR / "primary" / "participant_metrics.csv"
TRACK_A_SCENARIO_MANIFEST = TRACK_A_DIR / "primary" / "scenario_manifest.json"
MAPPING_REGISTRY = HERE / "config" / "placement_maps_v1.json"
FEATURE_BUILDER = HERE / "ablation_build.py"
CORE_MODULE = HERE / "locked_track_a" / "core.py"

PLAN_SCHEMA = "spinesense_section_7_5_config_compare_plan_v1"
ANALYSIS_ID = "spinesense_section_7_5_config_compare_2026-07-26"

ROLES = ["sacrum", "lower", "mid", "upper", "sternum"]
RANK = {role: index for index, role in enumerate(ROLES)}

# Inherited verbatim from the locked Track A primary scenario.
WEIGHT_SCHEME = "subject_only"
TUNING_MODE = "full_nested_13x12"
SEED = core.PRIMARY_SEED
MODELS = ["logistic", "rbf_svm"]
PRIMARY_MODEL = "logistic"
ROBUSTNESS_MODEL = "rbf_svm"

BOOTSTRAP_REPETITIONS = 10000
BOOTSTRAP_ALPHA = 0.05
ROW_MATCH_ATOL = 1e-12

# Reproduction targets, read back from the Track A archive rather than hard-coded here.
REPRODUCTION_CONFIG = "sacrum+lower+mid+upper+sternum"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_pair(a: str, b: str) -> str:
    low, high = (a, b) if RANK[a] < RANK[b] else (b, a)
    return f"pair_{low}_{high}"


def config_key(subset: tuple[str, ...]) -> str:
    return "+".join(role for role in ROLES if role in subset)


def channels_for(subset: tuple[str, ...]) -> tuple[str, list[str]]:
    """Return (representation, channel list) for one configuration.

    k == 1 uses the single-point channel of that location; k >= 2 uses every pair inside the
    configuration.  There is no fixed-root or star construction anywhere in this plan.
    """

    ordered = tuple(role for role in ROLES if role in subset)
    if len(ordered) == 1:
        return "single_point", [f"solo_{ordered[0]}"]
    return "all_pairs", [canonical_pair(a, b) for a, b in itertools.combinations(ordered, 2)]


def enumerate_configurations(features_per_channel: list[str]) -> list[dict]:
    configs: list[dict] = []
    for size in range(1, len(ROLES) + 1):
        for subset in itertools.combinations(ROLES, size):
            representation, channels = channels_for(subset)
            columns = [f"{channel}__{feature}" for channel in channels for feature in features_per_channel]
            configs.append(
                {
                    "config_key": config_key(subset),
                    "sensors": list(subset),
                    "k": len(subset),
                    "representation": representation,
                    "channels": channels,
                    "n_features": len(columns),
                    "feature_columns": columns,
                }
            )
    return configs


def focused_contrasts() -> list[dict]:
    """The four contrasts, chosen from the placement design before this run and fixed here."""

    return [
        {
            "contrast_id": "C1",
            "left": "sacrum+lower+mid+upper+sternum",
            "right": "sacrum",
            "direction": "left_minus_right",
            "design_reason": (
                "complete configuration against the pelvic root reference of Section 5.3, "
                "which is the principled single-point comparator"
            ),
            "dimension_matched": False,
        },
        {
            "contrast_id": "C2",
            "left": "sacrum+lower+mid+upper+sternum",
            "right": "sacrum+upper",
            "direction": "left_minus_right",
            "design_reason": (
                "complete configuration against the sacrum-to-upper-back bend span used by the "
                "Section 7.3 measurement validation"
            ),
            "dimension_matched": False,
        },
        {
            "contrast_id": "C3",
            "left": "sacrum+upper",
            "right": "sacrum",
            "direction": "left_minus_right",
            "design_reason": (
                "the only dimension-matched pair among the four: both carry 13 features, so "
                "fitting capacity is held fixed while sensor count, placement, and measured "
                "quantity still change together"
            ),
            "dimension_matched": True,
        },
        {
            "contrast_id": "C4",
            "left": "sacrum+lower+mid+upper+sternum",
            "right": "sacrum+lower+mid+upper",
            "direction": "left_minus_right",
            "design_reason": (
                "complete configuration against the four posterior locations, describing "
                "performance with and without the anterior sternum branch"
            ),
            "dimension_matched": False,
        },
    ]


def read_reproduction_targets() -> dict:
    import csv

    scalars: dict[str, float] = {}
    with TRACK_A_MODEL_SUMMARY.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if row["model"] in MODELS:
                scalars[row["model"]] = float(row["participant_first_macro_f1_mean"])
    missing = [model for model in MODELS if model not in scalars]
    if missing:
        raise SystemExit(f"Track A model summary is missing {missing}")
    return {
        "configuration": REPRODUCTION_CONFIG,
        "source": str(TRACK_A_MODEL_SUMMARY.relative_to(HERE)),
        "participant_first_macro_f1_mean": scalars,
        "scalar_tolerance": 1e-12,
        "compared_artefacts": [
            "every outer row prediction, keyed by row_id",
            "the selected configuration id of each of the 13 outer folds",
            "the 13-participant fixed-six macro F1 vector",
            "the participant-equal-weight macro F1 scalar",
        ],
        "policy": (
            "This is the first real run of the analysis, not a pre-flight check. Any mismatch "
            "aborts before the remaining 30 configurations start."
        ),
    }


def build_plan() -> dict:
    source_manifest = json.loads(SOURCE_MANIFEST.read_text(encoding="utf-8"))
    features_per_channel = list(source_manifest["features_per_channel"])
    dataset_manifest = json.loads(TRACK_A_DATASET_MANIFEST.read_text(encoding="utf-8"))
    scenario_manifest = json.loads(TRACK_A_SCENARIO_MANIFEST.read_text(encoding="utf-8"))

    if scenario_manifest["weight_scheme"] != WEIGHT_SCHEME:
        raise SystemExit(f"Track A primary weight scheme is {scenario_manifest['weight_scheme']!r}")
    if scenario_manifest["tuning_mode"] != TUNING_MODE:
        raise SystemExit(f"Track A primary tuning mode is {scenario_manifest['tuning_mode']!r}")
    if int(scenario_manifest["seed"]) != SEED:
        raise SystemExit(f"Track A primary seed is {scenario_manifest['seed']!r}")

    configs = enumerate_configurations(features_per_channel)
    if len(configs) != 31:
        raise SystemExit(f"expected 31 configurations, built {len(configs)}")
    dimensions = {config["config_key"]: config["n_features"] for config in configs}
    by_k: dict[int, set[int]] = {}
    for config in configs:
        by_k.setdefault(config["k"], set()).add(config["n_features"])
    if by_k != {1: {13}, 2: {13}, 3: {39}, 4: {78}, 5: {130}}:
        raise SystemExit(f"feature dimensions do not match Section 7.5: {by_k}")

    registry = core.build_model_registry()
    grids = {
        model: [config.as_dict() for config in registry[model]]
        for model in MODELS
    }

    contrasts = focused_contrasts()
    known = set(dimensions)
    for contrast in contrasts:
        for side in ("left", "right"):
            if contrast[side] not in known:
                raise SystemExit(f"{contrast['contrast_id']} references unknown config {contrast[side]!r}")
        if contrast["dimension_matched"] != (
            dimensions[contrast["left"]] == dimensions[contrast["right"]]
        ):
            raise SystemExit(f"{contrast['contrast_id']} dimension_matched flag disagrees with the plan")

    detailed = sorted({side for contrast in contrasts for side in (contrast["left"], contrast["right"])})

    return {
        "plan_schema": PLAN_SCHEMA,
        "analysis_id": ANALYSIS_ID,
        "manuscript_section": "7.5 传感器配置比较",
        "supersedes": {
            "document": "SpineSense Dissertation 大框架 v3-fable-2026-07-22.md",
            "superseded_scheme": (
                "fixed-root channels for k>1, logistic + random forest, pooled accuracy and "
                "per-class recall, and a ten-contrast participant-level Wilcoxon Holm family"
            ),
            "current_scheme": (
                "all-pairs channels for k>=2, logistic as primary with RBF-SVM as a "
                "model-family check, participant-equal-weight macro F1, and four focused "
                "contrasts reported as paired differences with percentile intervals only"
            ),
            "note": (
                "The blueprint paragraph is historical. Section 7.5 of the active manuscript "
                "is the authority for this run."
            ),
        },
        "inherited_from_locked_track_a": {
            "scenario": scenario_manifest["scenario_id"],
            "weight_scheme": WEIGHT_SCHEME,
            "tuning_mode": TUNING_MODE,
            "seed": SEED,
            "segment_list": "all rows of the frozen channel table; no quality-based exclusion",
            "preprocessing": (
                "median imputation, VarianceThreshold(0), and standardisation for scaled "
                "models, each fitted on training-fold participants only"
            ),
            "primary_metric": "participant-equal-weight fixed-six macro F1",
            "inner_selection_metric": "fixed-six macro F1 on the held-out inner participant",
            "selection_rule": (
                f"highest mean inner score, then the lowest simplicity rank within "
                f"{core.TIE_TOLERANCE} of the best"
            ),
            "sensitivity_scenarios": (
                "not re-run in Section 7.5; only the primary scenario is evaluated"
            ),
        },
        "models": {
            "primary": PRIMARY_MODEL,
            "robustness": ROBUSTNESS_MODEL,
            "order": MODELS,
            "grids": grids,
            "grid_sizes": {model: len(grids[model]) for model in MODELS},
        },
        "data_gate": {
            "source_table": str(SOURCE_TABLE.relative_to(HERE)),
            "source_table_sha256": sha256_file(SOURCE_TABLE),
            "source_manifest_sha256": sha256_file(SOURCE_MANIFEST),
            "source_manifest_binds_table": (
                source_manifest["outputs"]["channel_features.csv"] == sha256_file(SOURCE_TABLE)
            ),
            "track_a_input": str(TRACK_A_INPUT.relative_to(HERE)),
            "track_a_input_sha256": sha256_file(TRACK_A_INPUT),
            "track_a_dataset_manifest_sha256": sha256_file(TRACK_A_DATASET_MANIFEST),
            "checks": [
                "row identity: subject, trial_id, block, bout_index, y, and label agree "
                "row-for-row between the channel table and the Track A model matrix",
                "column identity: the 130 pair_* column names and their order agree with the "
                "Track A dataset manifest",
                f"numeric agreement: max absolute difference over the shared 130 columns "
                f"<= {ROW_MATCH_ATOL}",
                "single-point channels carry no NaN or infinity and no globally constant column",
            ],
            "row_match_atol": ROW_MATCH_ATOL,
            "expected_channels": list(dataset_manifest["channels"]),
            "features_per_channel": features_per_channel,
            "excluded_channels": ["legacy"],
        },
        "configurations": configs,
        "n_configurations": len(configs),
        "fold_split": {
            "outer": "leave one participant out, 13 folds, participants in sorted order",
            "inner": (
                "within the 12 training participants of each outer fold, leave one participant "
                "out again, 12 inner folds"
            ),
            "participants": list(scenario_manifest["participant_set"]),
        },
        "focused_contrasts": contrasts,
        "interval_method": {
            "statistic": "paired per-participant difference in fixed-six macro F1",
            "n_participants": 13,
            "resampling": "participant-level bootstrap, whole participants resampled with replacement",
            "repetitions": BOOTSTRAP_REPETITIONS,
            "seed": SEED,
            "alpha": BOOTSTRAP_ALPHA,
            "interval": "95% percentile interval",
            "significance_testing": "none; no sign-flip test, no Holm family, no p-values",
            "caveat": (
                "Outer training sets overlap heavily, so the interval understates variance and "
                "counts only as approximate within-cohort evidence."
            ),
        },
        "model_family_robustness": {
            "method": (
                "Spearman rank correlation between the primary and robustness models over the "
                "31 configuration scores, plus whether each focused contrast keeps its sign"
            ),
            "reported": ["spearman_rho", "per_contrast_sign_agreement"],
            "not_reported": ["correlation p-values", "any significance statement"],
        },
        "detailed_reporting": {
            "configurations": detailed,
            "artefacts": [
                "per-class recall",
                "participant-normalised confusion matrix",
            ],
            "reason": (
                "Section 7.5 restricts detailed reporting to the configurations entering the "
                "four focused contrasts."
            ),
        },
        "reproduction_first_run": read_reproduction_targets(),
        "code_hashes": {
            "config_compare_plan.py": sha256_file(Path(__file__).resolve()),
            "config_compare_eval.py": (
                sha256_file(HERE / "config_compare_eval.py")
                if (HERE / "config_compare_eval.py").exists()
                else None
            ),
            "ablation_build.py": sha256_file(FEATURE_BUILDER),
            "locked_track_a/core.py": sha256_file(CORE_MODULE),
            "config/placement_maps_v1.json": sha256_file(MAPPING_REGISTRY),
        },
        "outputs_required": [
            "run_plan.json and run_plan.sha256",
            "environment_lock.json",
            "reproduction_check.json",
            "checkpoints/<model>__<config_key>__<outer_participant>.json",
            "inner_scores.csv",
            "selected_configs.csv",
            "outer_predictions.csv",
            "participant_metrics.csv",
            "config_summary.csv",
            "config_results.json",
            "focused_contrasts.json",
            "model_family_robustness.json",
            "confusion/<config_key>__<model>.json",
            "ledgers/warnings_failures.jsonl and ledgers/runtime.jsonl",
            "run_manifest.json",
        ],
        "evidence_boundary": (
            "Scores describe the predictive performance of whole configurations on the "
            "pre-segmented six-class task. Sensor count, placement, single-point versus "
            "differential measurement, and feature dimensionality change together, so no "
            "result here establishes that a sensor is necessary, selects an optimal count, or "
            "fills any requirement status."
        ),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Freeze the Section 7.5 run plan.")
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument(
        "--force",
        action="store_true",
        help="Replace an existing read-only plan (records nothing; use only before any fits).",
    )
    args = parser.parse_args(argv)

    out_dir = args.out_dir.resolve()
    plan_path = out_dir / "run_plan.json"
    sidecar_path = out_dir / "run_plan.sha256"

    if plan_path.exists() and not args.force:
        raise SystemExit(f"plan already frozen: {plan_path} (pass --force to replace)")
    if (out_dir / "checkpoints").exists() and any((out_dir / "checkpoints").iterdir()):
        raise SystemExit("refusing to re-freeze a plan after checkpoints exist")

    out_dir.mkdir(parents=True, exist_ok=True)
    plan = build_plan()

    if not plan["data_gate"]["source_manifest_binds_table"]:
        raise SystemExit("ablation manifest does not bind channel_features.csv")

    for path in (plan_path, sidecar_path):
        if path.exists():
            path.chmod(stat.S_IWUSR | stat.S_IRUSR)
            path.unlink()

    payload = json.dumps(plan, indent=2, ensure_ascii=False) + "\n"
    plan_path.write_text(payload, encoding="utf-8")
    plan_sha = sha256_file(plan_path)
    sidecar_path.write_text(plan_sha + "\n", encoding="utf-8")

    read_only = stat.S_IRUSR | stat.S_IRGRP | stat.S_IROTH
    plan_path.chmod(read_only)
    sidecar_path.chmod(read_only)

    print(f"frozen plan     : {plan_path}")
    print(f"plan sha256     : {plan_sha}")
    print(f"configurations  : {plan['n_configurations']}")
    print(f"models          : {plan['models']['order']}")
    print(f"contrasts       : {[c['contrast_id'] for c in plan['focused_contrasts']]}")
    print(f"detailed configs: {plan['detailed_reporting']['configurations']}")
    print(
        "reproduction    : "
        f"{plan['reproduction_first_run']['configuration']} -> "
        f"{plan['reproduction_first_run']['participant_first_macro_f1_mean']}"
    )
    if os.name == "nt":
        print("note            : Windows read-only flag is advisory; do not edit the plan by hand.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
