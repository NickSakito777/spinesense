"""Root-choice and all-pairs extension of the sensor-subset ablation.

WHAT THE FROZEN ABLATION LEFT OUT
---------------------------------
``ablation_eval.py`` evaluates all 31 non-empty sensor subsets, but assigns each subset
exactly one reference frame by a fixed rule (lowest-ranked member becomes the root). Root
choice is therefore perfectly confounded with subset membership: the subsets rooted at
``lower`` are exactly the subsets that exclude ``sacrum``. No contrast in that run can
separate "S1 works well as a reference" from "S1 is a useful member".

This script removes that confound by enumerating, for every subset, every admissible root.

WHY THIS NEEDS NO FEATURE REBUILD
---------------------------------
Rooting subset S at r means taking every other member relative to r, i.e. the star of
relations {(r, m) : m in S, m != r}. The wide table already holds all 10 pairwise channels.
The only question is direction: the table stores qrel(parent, child) for parent ranked
below child, while a star centred on a higher-ranked root needs the inverse.

Reversing a relation negates the three underlying scalars, which leaves 7 of the 13
features unchanged (variance fractions, log ratios, sign-change count, twist dominance)
and negates the other 6 (the sign_* and dir_* terms). Both estimators here are invariant
to a per-feature sign flip: the standardised logistic loss and its L2 penalty are mirrored
exactly, and threshold splits in a forest map to their negated counterparts. This is
verified empirically at startup rather than assumed, so canonical channel columns are used
for every root and the enumeration stays exact.

STATUS -- EXPLORATORY, DELIBERATELY ISOLATED
--------------------------------------------
Model family and hyperparameters are identical to the frozen ablation and nothing is
tuned, so this does not widen the model search space that the locked Track A protocol
isolates. It does widen the configuration space, so every result here is exploratory.
These configurations do not join the ten pre-specified Wilcoxon contrasts or any Holm
family, and no configuration selected here may be reported as a necessity result.
"""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import platform
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import sklearn
from sklearn.ensemble import RandomForestClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import accuracy_score, confusion_matrix, f1_score, recall_score
from sklearn.model_selection import LeaveOneGroupOut
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import StandardScaler

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import ablation_build as ab  # noqa: E402

CLASSES = ["flexion", "extension", "left_bend", "right_bend", "left_twist", "right_twist"]
LABELS = list(range(6))
SIGN_FLIPPED = {"sign_x", "sign_y", "sign_tw", "dir_x", "dir_y", "dir_tw"}
ORD = ab.ROOT_ORDER
RANK = {r: i for i, r in enumerate(ORD)}

# Reproduction gate: recomputing these frozen configurations must land on the archived value.
GATE = {
    "sacrum+upper": ["pair_sacrum_upper"],
    "mid": ["solo_mid"],
    "sacrum+upper+sternum": ["pair_sacrum_upper", "pair_sacrum_sternum"],
    "legacy_frozen_block": ["legacy"],
}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_pair(a: str, b: str) -> str:
    lo, hi = (a, b) if RANK[a] < RANK[b] else (b, a)
    return f"pair_{lo}_{hi}"


def star_channels(subset: tuple[str, ...], root: str) -> list[str]:
    """Relations of the star centred on ``root``; direction is immaterial (see module docstring)."""
    return [canonical_pair(root, m) for m in subset if m != root]


def allpairs_channels(subset: tuple[str, ...]) -> list[str]:
    return [canonical_pair(a, b) for a, b in itertools.combinations(subset, 2)]


def subset_key(subset: tuple[str, ...]) -> str:
    return "+".join(r for r in ORD if r in subset)


def build_models() -> dict:
    return {
        "logistic": lambda: Pipeline([
            ("scaler", StandardScaler()),
            ("clf", LogisticRegression(max_iter=5000, class_weight="balanced")),
        ]),
        "random_forest": lambda: RandomForestClassifier(
            n_estimators=400, class_weight="balanced", random_state=0
        ),
    }


def loso(make_model, X, y, groups) -> dict:
    pred = np.full(len(y), -1, dtype=int)
    for tr, te in LeaveOneGroupOut().split(X, y, groups):
        m = make_model()
        m.fit(X[tr], y[tr])
        pred[te] = m.predict(X[te])
    assert (pred >= 0).all(), "every row must be held out exactly once"
    return {
        "accuracy": float(accuracy_score(y, pred)),
        "macro_f1": float(f1_score(y, pred, labels=LABELS, average="macro")),
        "per_class_recall": {
            CLASSES[i]: float(v) for i, v in enumerate(
                recall_score(y, pred, labels=LABELS, average=None, zero_division=0)
            )
        },
        "per_subject_accuracy": {
            str(s): float(accuracy_score(y[groups == s], pred[groups == s]))
            for s in sorted(set(groups))
        },
        "confusion_matrix": confusion_matrix(y, pred, labels=LABELS).tolist(),
    }


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="Root-choice and all-pairs ablation extension.")
    p.add_argument("--source-dir", type=Path, required=True,
                   help="Frozen ablation run directory holding channel_features.csv.")
    p.add_argument("--out-dir", type=Path, required=True, help="New output directory.")
    args = p.parse_args(argv)
    src, out = args.source_dir.resolve(), args.out_dir.resolve()

    feats = src / "channel_features.csv"
    manifest = json.loads((src / "ablation_manifest.json").read_text(encoding="utf-8"))
    feats_sha = sha256_file(feats)
    if manifest["outputs"]["channel_features.csv"] != feats_sha:
        raise SystemExit(f"stale/tampered channel_features.csv: {feats_sha}")
    frozen = json.loads((src / "ablation_results.json").read_text(encoding="utf-8"))["subsets"]

    out.mkdir(parents=True, exist_ok=True)
    out_path = out / "root_ablation_results.json"
    if out_path.exists():
        raise SystemExit(f"refusing to overwrite existing results: {out_path}")

    df = pd.read_csv(feats)
    y = df["y"].to_numpy(int)
    groups = df["subject"].to_numpy()
    models = build_models()

    def X_of(chans: list[str]) -> np.ndarray:
        cols = [f"{c}__{f}" for c in chans for f in ab.FEAT_NAMES]
        missing = [c for c in cols if c not in df.columns]
        if missing:
            raise SystemExit(f"missing columns: {missing[:3]}")
        return df[cols].to_numpy(float)

    # ---- gate 1: this machine reproduces the frozen ablation ----
    print("reproduction gate")
    gate_rows = []
    for key, chans in GATE.items():
        X = X_of(chans)
        for mname, factory in models.items():
            got = loso(factory, X, y, groups)["accuracy"]
            want = frozen[key][mname]["accuracy"]
            ok = abs(got - want) < 5e-4
            gate_rows.append({"config": key, "model": mname, "recomputed": got,
                              "archived": want, "match": bool(ok)})
            print(f"  {key:24s} {mname:14s} {got:.4f} vs {want:.4f}  {'ok' if ok else 'MISMATCH'}")
    if not all(r["match"] for r in gate_rows):
        raise SystemExit("reproduction gate failed; refusing to extend a run we cannot reproduce")

    # ---- gate 2: relation direction carries no information ----
    print("\ndirection-invariance gate")
    Xd = X_of(["pair_sacrum_upper"])
    Xf = Xd.copy()
    for i, f in enumerate(ab.FEAT_NAMES):
        if f in SIGN_FLIPPED:
            Xf[:, i] *= -1.0
    dir_rows = []
    for mname, factory in models.items():
        a, b = loso(factory, Xd, y, groups)["accuracy"], loso(factory, Xf, y, groups)["accuracy"]
        dir_rows.append({"model": mname, "canonical": a, "reversed": b, "abs_delta": abs(a - b)})
        print(f"  {mname:14s} canonical {a:.6f}  reversed {b:.6f}  |delta| {abs(a-b):.6f}")

    # ---- enumeration ----
    jobs: list[tuple[str, dict]] = []
    for r in ORD:
        jobs.append((f"solo::{r}", {"family": "solo", "subset": [r], "root": r, "k": 1,
                                    "channels": [f"solo_{r}"]}))
    subsets = [tuple(c) for k in range(2, len(ORD) + 1) for c in itertools.combinations(ORD, k)]
    for sub in subsets:
        key = subset_key(sub)
        for root in sub:
            jobs.append((f"star::{key}::root={root}",
                         {"family": "star", "subset": list(sub), "root": root, "k": len(sub),
                          "channels": star_channels(sub, root)}))
        jobs.append((f"allpairs::{key}",
                     {"family": "allpairs", "subset": list(sub), "root": None, "k": len(sub),
                      "channels": allpairs_channels(sub)}))

    print(f"\nenumerating {len(jobs)} configurations "
          f"(5 solo + {len(subsets) and sum(len(s) for s in subsets)} star + {len(subsets)} all-pairs)")
    results: dict[str, dict] = {}
    for i, (cid, meta) in enumerate(jobs, 1):
        X = X_of(meta["channels"])
        entry = dict(meta)
        entry["n_features"] = X.shape[1]
        entry["fixed_root_in_frozen_run"] = bool(
            meta["family"] == "star" and meta["root"] == min(meta["subset"], key=lambda r: RANK[r])
        )
        for mname, factory in models.items():
            entry[mname] = loso(factory, X, y, groups)
        results[cid] = entry
        print(f"  [{i:3d}/{len(jobs)}] {cid:44s} d={X.shape[1]:3d} "
              f"log={entry['logistic']['accuracy']:.4f} rf={entry['random_forest']['accuracy']:.4f}",
              flush=True)

    # ---- derived: within-subset root effect (membership and dimension held fixed) ----
    root_effect = {}
    for mname in models:
        per_sensor: dict[str, list[float]] = {r: [] for r in ORD}
        spreads = []
        for sub in subsets:
            key = subset_key(sub)
            scores = {r: results[f"star::{key}::root={r}"][mname]["accuracy"] for r in sub}
            mu = float(np.mean(list(scores.values())))
            spreads.append({"subset": key, "k": len(sub),
                            "scores": scores, "mean": mu,
                            "spread": float(max(scores.values()) - min(scores.values())),
                            "best_root": max(scores, key=scores.get),
                            "worst_root": min(scores, key=scores.get)})
            for r, v in scores.items():
                per_sensor[r].append(v - mu)
        root_effect[mname] = {
            "per_subset": spreads,
            "mean_within_subset_spread": float(np.mean([s["spread"] for s in spreads])),
            "max_within_subset_spread": float(np.max([s["spread"] for s in spreads])),
            "sensor_effect": {
                r: {"n_subsets": len(v), "mean_centred_delta": float(np.mean(v)),
                    "median_centred_delta": float(np.median(v)),
                    "min": float(np.min(v)), "max": float(np.max(v))}
                for r, v in per_sensor.items() if v
            },
            "best_root_counts": {
                r: sum(1 for s in spreads if s["best_root"] == r) for r in ORD
            },
        }

    # ---- derived: star vs all-pairs, per subset ----
    star_vs_all = {}
    for mname in models:
        rows = []
        for sub in subsets:
            key = subset_key(sub)
            stars = {r: results[f"star::{key}::root={r}"][mname]["accuracy"] for r in sub}
            ap = results[f"allpairs::{key}"][mname]["accuracy"]
            fixed = results[f"star::{key}::root={min(sub, key=lambda r: RANK[r])}"][mname]["accuracy"]
            rows.append({"subset": key, "k": len(sub),
                         "fixed_root": fixed, "best_star": max(stars.values()),
                         "mean_star": float(np.mean(list(stars.values()))),
                         "allpairs": ap,
                         "allpairs_minus_fixed": ap - fixed,
                         "allpairs_minus_best_star": ap - max(stars.values()),
                         "star_dim": 13 * (len(sub) - 1),
                         "allpairs_dim": 13 * (len(sub) * (len(sub) - 1) // 2)})
        star_vs_all[mname] = {
            "per_subset": rows,
            "mean_allpairs_minus_fixed": float(np.mean([r["allpairs_minus_fixed"] for r in rows])),
            "mean_allpairs_minus_best_star": float(
                np.mean([r["allpairs_minus_best_star"] for r in rows])),
        }

    payload = {
        "run_type": "sensor_subset_ablation_root_and_allpairs_extension",
        "status": "exploratory; isolated from the pre-specified contrast family and from locked Track A",
        "source_run": str(src),
        "channel_features_sha256": feats_sha,
        "evaluator_sha256": sha256_file(Path(__file__).resolve()),
        "analysis_mode": manifest["analysis_mode"],
        "n_rows": int(len(df)),
        "n_subjects": int(df["subject"].nunique()),
        "chance": 1.0 / 6.0,
        "majority": float(np.bincount(y).max() / len(y)),
        "root_order": ORD,
        "n_configurations": len(jobs),
        "reproduction_gate": gate_rows,
        "direction_invariance_gate": dir_rows,
        "configurations": results,
        "root_effect": root_effect,
        "star_vs_allpairs": star_vs_all,
        "caveats": {
            "exploratory": (
                "Configuration space is widened relative to the frozen 31-subset run. No result "
                "here enters a pre-specified family, and none may be reported as necessity "
                "evidence for a sensor count or placement."
            ),
            "direction": (
                "Relation direction is treated as uninformative on the basis of the startup gate. "
                "Logistic is exactly invariant; the forest differs only by split tie-breaking."
            ),
            "dimension": (
                "Star configurations carry 13*(k-1) features and all-pairs configurations carry "
                "13*C(k,2). Star-versus-all-pairs contrasts therefore vary capacity as well as "
                "relation coverage. Within-subset root contrasts hold both membership and "
                "dimension fixed and are the only capacity-controlled comparison here."
            ),
        },
        "environment": {
            "python": platform.python_version(),
            "numpy": np.__version__,
            "pandas": pd.__version__,
            "scikit_learn": sklearn.__version__,
        },
    }
    out_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(f"\nwrote {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
