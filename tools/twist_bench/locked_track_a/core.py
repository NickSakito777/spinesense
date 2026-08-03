"""Core estimators, metrics, weights, grids, and fit helpers for locked Track A.

This module deliberately contains no outer-loop orchestration.  It is kept small enough for
the P0 code hash and the P1 synthetic unit tests to audit independently of real cohort scores.
"""

from __future__ import annotations

import itertools
import json
import math
import os
import pickle
import resource
import sys
import time
import warnings
from dataclasses import dataclass
from typing import Any, Iterable, Sequence

# Prevent nested BLAS/OpenMP oversubscription.  The outer runner may parallelise independent
# inner folds; every individual fit is deliberately single-threaded except where sklearn has
# no thread control knob.
for _name in (
    "OMP_NUM_THREADS",
    "OPENBLAS_NUM_THREADS",
    "MKL_NUM_THREADS",
    "VECLIB_MAXIMUM_THREADS",
    "NUMEXPR_NUM_THREADS",
):
    os.environ[_name] = "1"

import numpy as np
from scipy import linalg
from sklearn.base import BaseEstimator, ClassifierMixin
from sklearn.dummy import DummyClassifier
from sklearn.ensemble import HistGradientBoostingClassifier, RandomForestClassifier
from sklearn.feature_selection import VarianceThreshold
from sklearn.impute import SimpleImputer
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import confusion_matrix, f1_score, precision_recall_fscore_support
from sklearn.preprocessing import StandardScaler
from sklearn.svm import LinearSVC, SVC
from sklearn.utils.validation import check_array, check_is_fitted, check_X_y


LABEL_NAMES = [
    "flexion",
    "extension",
    "left_bend",
    "right_bend",
    "left_twist",
    "right_twist",
]
LABELS = np.arange(len(LABEL_NAMES), dtype=int)
LABEL_TO_INT = {name: i for i, name in enumerate(LABEL_NAMES)}
PRIMARY_SEED = 20260721
RF_HGB_SEEDS = list(range(20260721, 20260726))
LOGISTIC_SEEDS = list(range(20260721, 20260724))
TIE_TOLERANCE = 0.01
NUMERIC_TIE_TOL = 1e-12
N_FIXED_FEATURES = 130

MODEL_ORDER = [
    "logistic",
    "linear_svm",
    "rbf_svm",
    "weighted_shrinkage_lda",
    "random_forest",
    "hist_gradient_boosting",
    "dummy",
]
FORMAL_MODELS = [
    "logistic",
    "rbf_svm",
    "weighted_shrinkage_lda",
    "random_forest",
]
APPENDIX_MODELS = ["linear_svm", "hist_gradient_boosting", "dummy"]
FORMAL_CHALLENGERS = ["rbf_svm", "weighted_shrinkage_lda", "random_forest"]
SCALED_MODELS = {
    "logistic",
    "linear_svm",
    "rbf_svm",
    "weighted_shrinkage_lda",
}


def canonical_json(value: Any) -> str:
    """Stable JSON representation used in config IDs and manifests."""

    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)


class WeightedShrinkageLDA(ClassifierMixin, BaseEstimator):
    """LDA with observation weights, uniform priors, and fixed covariance shrinkage.

    The covariance contract intentionally mirrors sklearn's ``solver='lsqr'`` convention for
    equal weights and explicit uniform priors:

    1. compute an MLE covariance within every class (denominator = class weight sum),
    2. average class covariances with uniform class priors,
    3. shrink the pooled covariance toward ``trace(S) / p * I``.

    The estimator performs no imputation or scaling; the runner fits those transformations on
    the corresponding training participants only.
    """

    def __init__(self, shrinkage: float = 0.2):
        self.shrinkage = shrinkage

    def fit(self, X: np.ndarray, y: np.ndarray, sample_weight: np.ndarray | None = None):
        X, y = check_X_y(X, y, dtype=np.float64, ensure_min_samples=2)
        lam = float(self.shrinkage)
        if not 0.0 <= lam <= 1.0:
            raise ValueError("shrinkage must lie in [0, 1]")

        if sample_weight is None:
            w = np.ones(X.shape[0], dtype=np.float64)
        else:
            w = np.asarray(sample_weight, dtype=np.float64)
            if w.ndim != 1 or w.shape[0] != X.shape[0]:
                raise ValueError("sample_weight must be a 1D array with one value per row")
            if not np.isfinite(w).all() or np.any(w <= 0.0):
                raise ValueError("sample_weight must be finite and strictly positive")

        classes = np.unique(y)
        if classes.size < 2:
            raise ValueError("WeightedShrinkageLDA requires at least two classes")

        n_features = X.shape[1]
        means = np.empty((classes.size, n_features), dtype=np.float64)
        class_covariances = np.empty((classes.size, n_features, n_features), dtype=np.float64)
        class_weight_sums = np.empty(classes.size, dtype=np.float64)

        for j, cls in enumerate(classes):
            mask = y == cls
            Xc = X[mask]
            wc = w[mask]
            Wc = float(wc.sum())
            if Wc <= 0.0:
                raise ValueError(f"class {cls!r} has zero total weight")
            mu = np.sum(Xc * wc[:, None], axis=0) / Wc
            centered = Xc - mu
            cov = (centered * wc[:, None]).T @ centered / Wc
            means[j] = mu
            class_covariances[j] = cov
            class_weight_sums[j] = Wc

        priors = np.full(classes.size, 1.0 / classes.size, dtype=np.float64)
        pooled = np.tensordot(priors, class_covariances, axes=(0, 0))
        target_scale = float(np.trace(pooled) / n_features)
        if not np.isfinite(target_scale) or target_scale <= 0.0:
            raise ValueError("pooled within-class covariance has non-positive trace")
        shrunk = (1.0 - lam) * pooled + lam * target_scale * np.eye(n_features)

        # sklearn's lsqr implementation uses scipy.linalg.lstsq.  Using the same operation is
        # important for the equal-weight parity gate on correlated/high-dimensional matrices.
        coef = linalg.lstsq(shrunk, means.T)[0].T
        intercept = -0.5 * np.sum(means * coef, axis=1) + np.log(priors)

        self.classes_ = classes
        self.priors_ = priors
        self.means_ = means
        self.class_covariances_ = class_covariances
        self.class_weight_sums_ = class_weight_sums
        self.covariance_ = shrunk
        self.coef_ = coef
        self.intercept_ = intercept
        self.n_features_in_ = n_features
        return self

    def decision_function(self, X: np.ndarray) -> np.ndarray:
        check_is_fitted(self, ("coef_", "intercept_", "classes_"))
        X = check_array(X, dtype=np.float64)
        return X @ self.coef_.T + self.intercept_

    def predict(self, X: np.ndarray) -> np.ndarray:
        scores = self.decision_function(X)
        return self.classes_[np.argmax(scores, axis=1)]

    def predict_proba(self, X: np.ndarray) -> np.ndarray:
        scores = self.decision_function(X)
        scores = scores - scores.max(axis=1, keepdims=True)
        exp_scores = np.exp(scores)
        return exp_scores / exp_scores.sum(axis=1, keepdims=True)


@dataclass(frozen=True)
class ModelConfig:
    model: str
    config_id: str
    simplicity_rank: int
    params: dict[str, Any]

    def as_dict(self) -> dict[str, Any]:
        return {
            "model": self.model,
            "config_id": self.config_id,
            "simplicity_rank": self.simplicity_rank,
            "params": self.params,
        }


def _configs(model: str, params_in_simplicity_order: Iterable[dict[str, Any]]) -> list[ModelConfig]:
    out: list[ModelConfig] = []
    for rank, params in enumerate(params_in_simplicity_order):
        out.append(
            ModelConfig(
                model=model,
                config_id=f"{model}:{rank:02d}",
                simplicity_rank=rank,
                params=dict(params),
            )
        )
    return out


def build_model_registry() -> dict[str, list[ModelConfig]]:
    """Return the complete finite grids in their pre-result simplicity order."""

    logistic: list[dict[str, Any]] = [
        {"family": "l2", "C": C} for C in (0.01, 0.1, 1.0, 10.0)
    ]
    # L2 is preferred under a practical tie.  Within elastic net, lower C and then lower
    # l1_ratio are treated as simpler by the pre-result ordering.
    logistic.extend(
        {"family": "elastic_net", "C": C, "l1_ratio": ratio}
        for C in (0.03, 0.3, 3.0)
        for ratio in (0.25, 0.50, 0.75)
    )

    linear = [{"C": C} for C in (0.01, 0.1, 1.0, 10.0)]
    rbf = [
        {"C": C, "gamma_multiplier": mult}
        for C in (0.1, 1.0, 10.0, 100.0)
        for mult in (0.25, 1.0, 4.0)
    ]
    lda = [{"shrinkage": lam} for lam in (0.95, 0.80, 0.50, 0.20, 0.05)]
    rf = [
        {
            "n_estimators": 1000,
            "max_depth": depth,
            "min_samples_leaf": leaf,
            "max_features": max_features,
        }
        for depth in (4, 8, 12)
        for leaf in (15, 5)
        for max_features in ("sqrt", 0.30)
    ]

    schedules = [(0.10, 100), (0.05, 200), (0.03, 300)]
    hgb = [
        {
            "learning_rate": lr,
            "max_iter": n_iter,
            "max_depth": depth,
            "l2_regularization": l2,
            "min_samples_leaf": 20,
            "early_stopping": False,
        }
        for depth in (2, 3)
        for l2 in (10.0, 1.0)
        for lr, n_iter in schedules
    ]

    return {
        "logistic": _configs("logistic", logistic),
        "linear_svm": _configs("linear_svm", linear),
        "rbf_svm": _configs("rbf_svm", rbf),
        "weighted_shrinkage_lda": _configs("weighted_shrinkage_lda", lda),
        "random_forest": _configs("random_forest", rf),
        "hist_gradient_boosting": _configs("hist_gradient_boosting", hgb),
        "dummy": _configs("dummy", [{"strategy": "most_frequent"}]),
    }


def registry_as_jsonable(registry: dict[str, list[ModelConfig]] | None = None) -> dict[str, Any]:
    if registry is None:
        registry = build_model_registry()
    return {
        "model_order": MODEL_ORDER,
        "formal_models": FORMAL_MODELS,
        "appendix_models": APPENDIX_MODELS,
        "formal_challengers": FORMAL_CHALLENGERS,
        "practical_tie_tolerance": TIE_TOLERANCE,
        "rbf_gamma_denominator": "training-fold p after VarianceThreshold(0)",
        "simplicity_interpretation": {
            "logistic": "L2 before elastic net; C ascending; elastic l1_ratio ascending",
            "linear_svm": "C ascending",
            "rbf_svm": "C ascending, then gamma multiplier ascending",
            "weighted_shrinkage_lda": "shrinkage descending (.95 to .05)",
            "random_forest": "depth ascending, min leaf descending, max features sqrt before .30",
            "hist_gradient_boosting": (
                "depth ascending, L2 descending, then (.10,100), (.05,200), (.03,300)"
            ),
            "dummy": "most_frequent only",
        },
        "grids": {
            name: [config.as_dict() for config in configs]
            for name, configs in registry.items()
        },
    }


def make_estimator(
    model: str,
    params: dict[str, Any],
    seed: int = PRIMARY_SEED,
    n_features_after_variance: int | None = None,
):
    """Construct exactly one locked estimator; no class weights are ever added."""

    if model == "logistic":
        family = params["family"]
        if family == "l2":
            return LogisticRegression(
                C=float(params["C"]),
                solver="lbfgs",
                l1_ratio=0.0,
                max_iter=10000,
                tol=1e-4,
                class_weight=None,
                random_state=seed,
            )
        if family == "elastic_net":
            return LogisticRegression(
                C=float(params["C"]),
                solver="saga",
                l1_ratio=float(params["l1_ratio"]),
                max_iter=10000,
                tol=1e-4,
                class_weight=None,
                random_state=seed,
            )
        raise ValueError(f"unknown logistic family: {family}")
    if model == "linear_svm":
        return LinearSVC(
            C=float(params["C"]),
            penalty="l2",
            loss="squared_hinge",
            dual="auto",
            multi_class="ovr",
            class_weight=None,
            random_state=seed,
            max_iter=20000,
            tol=1e-4,
        )
    if model == "rbf_svm":
        if n_features_after_variance is None or n_features_after_variance <= 0:
            raise ValueError("RBF-SVM requires the positive training-fold post-threshold p")
        return SVC(
            C=float(params["C"]),
            gamma=float(params["gamma_multiplier"]) / int(n_features_after_variance),
            kernel="rbf",
            class_weight=None,
            shrinking=True,
            tol=1e-3,
            cache_size=1024,
            random_state=seed,
        )
    if model == "weighted_shrinkage_lda":
        return WeightedShrinkageLDA(shrinkage=float(params["shrinkage"]))
    if model == "random_forest":
        return RandomForestClassifier(
            n_estimators=int(params["n_estimators"]),
            max_depth=int(params["max_depth"]),
            min_samples_leaf=int(params["min_samples_leaf"]),
            max_features=params["max_features"],
            class_weight=None,
            bootstrap=True,
            random_state=seed,
            n_jobs=1,
        )
    if model == "hist_gradient_boosting":
        return HistGradientBoostingClassifier(
            learning_rate=float(params["learning_rate"]),
            max_iter=int(params["max_iter"]),
            max_depth=int(params["max_depth"]),
            l2_regularization=float(params["l2_regularization"]),
            min_samples_leaf=int(params["min_samples_leaf"]),
            early_stopping=False,
            class_weight=None,
            random_state=seed,
        )
    if model == "dummy":
        return DummyClassifier(strategy="most_frequent", random_state=seed)
    raise ValueError(f"unknown locked model: {model}")


def compute_training_weights(
    subjects: Sequence[str],
    y: Sequence[int],
    scheme: str,
) -> np.ndarray:
    """Compute weights using only the rows in the current training fold."""

    subjects = np.asarray(subjects)
    y = np.asarray(y)
    if subjects.ndim != 1 or y.ndim != 1 or subjects.size != y.size:
        raise ValueError("subjects and y must be aligned 1D arrays")
    if subjects.size == 0:
        raise ValueError("cannot weight an empty training fold")

    if scheme == "unweighted":
        raw = np.ones(subjects.size, dtype=np.float64)
    elif scheme == "subject_only":
        _, inverse, counts = np.unique(subjects, return_inverse=True, return_counts=True)
        raw = 1.0 / counts[inverse].astype(np.float64)
    elif scheme == "capped_subject_class":
        keys = np.asarray([f"{s}\x1f{int(c)}" for s, c in zip(subjects, y)], dtype=object)
        _, inverse, counts = np.unique(keys, return_inverse=True, return_counts=True)
        raw = 1.0 / counts[inverse].astype(np.float64)
        cap = 3.0 * float(np.median(raw))
        raw = np.minimum(raw, cap)
    else:
        raise ValueError(f"unknown weight scheme: {scheme}")

    weights = raw / float(np.mean(raw))
    if not np.isfinite(weights).all() or np.any(weights <= 0.0):
        raise AssertionError("weight normalization produced invalid values")
    return weights


class FittedProcedure:
    """Serializable fitted preprocessing + classifier used by every fold and deployment fit."""

    def __init__(self, model: str, params: dict[str, Any], seed: int):
        self.model = model
        self.params = dict(params)
        self.seed = int(seed)
        self.imputer = SimpleImputer(strategy="median")
        self.variance = VarianceThreshold(threshold=0.0)
        self.scaler = StandardScaler() if model in SCALED_MODELS else None
        self.estimator = None

    def fit(self, X: np.ndarray, y: np.ndarray, sample_weight: np.ndarray):
        X = np.asarray(X, dtype=np.float64)
        Xt = self.imputer.fit_transform(X)
        Xt = self.variance.fit_transform(Xt)
        if Xt.shape[1] == 0:
            raise ValueError("VarianceThreshold removed every feature")
        if self.scaler is not None:
            Xt = self.scaler.fit_transform(Xt)
        self.estimator = make_estimator(
            self.model,
            self.params,
            self.seed,
            n_features_after_variance=int(Xt.shape[1]),
        )
        self.estimator.fit(Xt, y, sample_weight=sample_weight)
        self.n_features_in_ = X.shape[1]
        self.n_features_after_variance_ = Xt.shape[1]
        return self

    def transform(self, X: np.ndarray) -> np.ndarray:
        Xt = self.imputer.transform(np.asarray(X, dtype=np.float64))
        Xt = self.variance.transform(Xt)
        if self.scaler is not None:
            Xt = self.scaler.transform(Xt)
        return Xt

    def predict(self, X: np.ndarray) -> np.ndarray:
        if self.estimator is None:
            raise RuntimeError("procedure has not been fitted")
        return self.estimator.predict(self.transform(X))

    def serialized_size_bytes(self) -> int:
        return len(pickle.dumps(self, protocol=pickle.HIGHEST_PROTOCOL))


def fixed_six_metrics(y_true: Sequence[int], y_pred: Sequence[int]) -> dict[str, Any]:
    y_true = np.asarray(y_true, dtype=int)
    y_pred = np.asarray(y_pred, dtype=int)
    if y_true.shape != y_pred.shape or y_true.ndim != 1:
        raise ValueError("y_true and y_pred must be aligned 1D arrays")
    precision, recall, f1, support = precision_recall_fscore_support(
        y_true,
        y_pred,
        labels=LABELS,
        zero_division=0,
    )
    cm = confusion_matrix(y_true, y_pred, labels=LABELS)
    return {
        "fixed6_macro_f1": float(np.mean(f1)),
        "fixed6_balanced_accuracy": float(np.mean(recall)),
        "accuracy": float(np.mean(y_true == y_pred)),
        "precision": precision.astype(float),
        "recall": recall.astype(float),
        "f1": f1.astype(float),
        "support": support.astype(int),
        "confusion": cm.astype(int),
    }


def participant_first_macro_f1(
    y_true: Sequence[int],
    y_pred: Sequence[int],
    participants: Sequence[str],
) -> tuple[float, dict[str, float]]:
    y_true = np.asarray(y_true, dtype=int)
    y_pred = np.asarray(y_pred, dtype=int)
    participants = np.asarray(participants)
    if not (y_true.shape == y_pred.shape == participants.shape):
        raise ValueError("inputs must have the same shape")
    per_participant: dict[str, float] = {}
    for participant in sorted(np.unique(participants)):
        mask = participants == participant
        per_participant[str(participant)] = fixed_six_metrics(
            y_true[mask], y_pred[mask]
        )["fixed6_macro_f1"]
    return float(np.mean(list(per_participant.values()))), per_participant


def participant_normalized_confusion(
    y_true: Sequence[int],
    y_pred: Sequence[int],
    participants: Sequence[str],
) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    y_true = np.asarray(y_true, dtype=int)
    y_pred = np.asarray(y_pred, dtype=int)
    participants = np.asarray(participants)
    matrices: dict[str, np.ndarray] = {}
    for participant in sorted(np.unique(participants)):
        mask = participants == participant
        cm = confusion_matrix(y_true[mask], y_pred[mask], labels=LABELS).astype(float)
        row_sums = cm.sum(axis=1, keepdims=True)
        if np.any(row_sums == 0):
            raise ValueError(f"participant {participant} is missing a fixed class")
        matrices[str(participant)] = cm / row_sums
    return np.mean(np.stack(list(matrices.values())), axis=0), matrices


def select_config(
    configs: Sequence[ModelConfig],
    mean_scores: dict[str, float],
) -> tuple[ModelConfig, dict[str, Any]]:
    expected_ids = {config.config_id for config in configs}
    actual_ids = set(mean_scores)
    if actual_ids != expected_ids:
        raise RuntimeError(
            "inner score registry is incomplete or contains unknown configurations: "
            f"missing={sorted(expected_ids - actual_ids)} extra={sorted(actual_ids - expected_ids)}"
        )
    eligible = [c for c in configs if np.isfinite(mean_scores[c.config_id])]
    if not eligible:
        raise RuntimeError("no eligible configuration has a finite complete inner score")
    best_score = max(mean_scores[c.config_id] for c in eligible)
    tied = [c for c in eligible if best_score - mean_scores[c.config_id] <= TIE_TOLERANCE + 1e-15]
    selected = min(tied, key=lambda c: c.simplicity_rank)
    return selected, {
        "best_mean_score": float(best_score),
        "selected_mean_score": float(mean_scores[selected.config_id]),
        "tie_threshold": float(best_score - TIE_TOLERANCE),
        "practically_tied_config_ids": [c.config_id for c in tied],
    }


def exhaustive_sign_flip(differences: Sequence[float]) -> dict[str, Any]:
    """Two-sided exhaustive sign-flip test using abs(mean(delta)), with no +1 correction."""

    d = np.asarray(differences, dtype=np.float64)
    if d.ndim != 1 or d.size == 0 or not np.isfinite(d).all():
        raise ValueError("differences must be a finite non-empty 1D vector")
    observed = abs(float(np.mean(d)))
    extreme = 0
    total = 1 << d.size
    for bits in range(total):
        signs = np.fromiter(
            (1.0 if (bits >> j) & 1 else -1.0 for j in range(d.size)),
            dtype=np.float64,
            count=d.size,
        )
        statistic = abs(float(np.mean(signs * d)))
        if statistic >= observed - 1e-15:
            extreme += 1
    return {
        "statistic": "abs_mean",
        "observed": observed,
        "n_participants": int(d.size),
        "total_assignments": int(total),
        "extreme_assignments": int(extreme),
        "p_value": float(extreme / total),
        "plus_one_correction": False,
    }


def holm_adjust(p_values: Sequence[float]) -> np.ndarray:
    p = np.asarray(p_values, dtype=np.float64)
    if p.ndim != 1 or p.size == 0 or not np.isfinite(p).all():
        raise ValueError("p_values must be a finite non-empty 1D vector")
    if np.any((p < 0.0) | (p > 1.0)):
        raise ValueError("p_values must lie in [0, 1]")
    order = np.argsort(p, kind="mergesort")
    sorted_p = p[order]
    adjusted_sorted = np.maximum.accumulate((p.size - np.arange(p.size)) * sorted_p)
    adjusted_sorted = np.minimum(adjusted_sorted, 1.0)
    adjusted = np.empty_like(adjusted_sorted)
    adjusted[order] = adjusted_sorted
    return adjusted


def bootstrap_mean_ci(
    differences: Sequence[float],
    indices: np.ndarray,
    alpha: float = 0.05,
) -> tuple[float, float]:
    d = np.asarray(differences, dtype=np.float64)
    indices = np.asarray(indices, dtype=int)
    if indices.ndim != 2 or indices.shape[1] != d.size:
        raise ValueError("indices must have shape (repetitions, n_participants)")
    means = np.mean(d[indices], axis=1)
    low, high = np.percentile(means, [100 * alpha / 2, 100 * (1 - alpha / 2)])
    return float(low), float(high)


def make_bootstrap_indices(n_participants: int, repetitions: int = 10000, seed: int = PRIMARY_SEED) -> np.ndarray:
    if n_participants <= 0 or repetitions <= 0:
        raise ValueError("n_participants and repetitions must be positive")
    return np.random.default_rng(seed).integers(
        0, n_participants, size=(repetitions, n_participants), endpoint=False
    )


def _warning_records(caught: Sequence[warnings.WarningMessage]) -> list[dict[str, str]]:
    return [
        {
            "category": item.category.__name__,
            "message": str(item.message),
            "filename": str(item.filename),
            "lineno": str(item.lineno),
        }
        for item in caught
    ]


def peak_rss_bytes() -> int:
    """Return the process peak RSS using the platform-specific getrusage unit."""

    value = int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
    return value if sys.platform == "darwin" else value * 1024


def fit_predict_once(
    model: str,
    params: dict[str, Any],
    seed: int,
    X_train: np.ndarray,
    y_train: np.ndarray,
    subjects_train: np.ndarray,
    X_test: np.ndarray,
    weight_scheme: str,
    return_procedure: bool = False,
    measure_size: bool = False,
) -> dict[str, Any]:
    """Fit one train/test split with captured warnings, failure, runtime, and memory size."""

    started = time.perf_counter()
    fit_seconds = math.nan
    predict_seconds = math.nan
    procedure: FittedProcedure | None = None
    caught: Sequence[warnings.WarningMessage] = []
    weights: np.ndarray | None = None
    try:
        weights = compute_training_weights(subjects_train, y_train, weight_scheme)
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            procedure = FittedProcedure(model, params, seed)
            fit_started = time.perf_counter()
            try:
                procedure.fit(X_train, y_train, weights)
            finally:
                fit_seconds = time.perf_counter() - fit_started
            pred_started = time.perf_counter()
            try:
                prediction = np.asarray(procedure.predict(X_test), dtype=int)
            finally:
                predict_seconds = time.perf_counter() - pred_started
        result = {
            "status": "warning" if caught else "ok",
            "prediction": prediction,
            "warnings": _warning_records(caught),
            "failure_type": "",
            "failure_message": "",
            "fit_seconds": float(fit_seconds),
            "predict_seconds": float(predict_seconds),
            "total_seconds": float(time.perf_counter() - started),
            "process_peak_rss_bytes": peak_rss_bytes(),
            "n_features_after_variance": int(procedure.n_features_after_variance_),
            "weight_min": float(weights.min()),
            "weight_max": float(weights.max()),
            "weight_mean": float(weights.mean()),
            "serialized_size_bytes": (
                int(procedure.serialized_size_bytes()) if measure_size else None
            ),
        }
        if return_procedure:
            result["procedure"] = procedure
        return result
    except Exception as exc:  # returned to the caller and ledger; never silently converted to score
        return {
            "status": "failed",
            "prediction": None,
            "warnings": _warning_records(caught),
            "failure_type": type(exc).__name__,
            "failure_message": str(exc),
            "fit_seconds": float(fit_seconds),
            "predict_seconds": float(predict_seconds),
            "total_seconds": float(time.perf_counter() - started),
            "process_peak_rss_bytes": peak_rss_bytes(),
            "n_features_after_variance": getattr(
                procedure, "n_features_after_variance_", None
            ),
            "weight_min": float(weights.min()) if weights is not None else None,
            "weight_max": float(weights.max()) if weights is not None else None,
            "weight_mean": float(weights.mean()) if weights is not None else None,
            "serialized_size_bytes": None,
        }


def validate_registry_counts(registry: dict[str, list[ModelConfig]] | None = None) -> None:
    if registry is None:
        registry = build_model_registry()
    expected = {
        "logistic": 13,
        "linear_svm": 4,
        "rbf_svm": 12,
        "weighted_shrinkage_lda": 5,
        "random_forest": 12,
        "hist_gradient_boosting": 12,
        "dummy": 1,
    }
    actual = {name: len(configs) for name, configs in registry.items()}
    if actual != expected:
        raise AssertionError(f"locked registry count mismatch: {actual} != {expected}")


validate_registry_counts()
