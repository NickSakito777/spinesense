"""P1 synthetic tests for the locked Track A implementation.

These tests never read the cohort CSV and therefore cannot expose or influence an outer score.
"""

from __future__ import annotations

import hashlib
import pickle
import tempfile
import unittest
import warnings
from unittest import mock

import joblib
import numpy as np
from sklearn.base import clone
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
from sklearn.metrics import f1_score

from .core import (
    LABELS,
    FittedProcedure,
    WeightedShrinkageLDA,
    bootstrap_mean_ci,
    build_model_registry,
    compute_training_weights,
    exhaustive_sign_flip,
    fixed_six_metrics,
    fit_predict_once,
    holm_adjust,
    make_bootstrap_indices,
    participant_first_macro_f1,
    participant_normalized_confusion,
    select_config,
    validate_registry_counts,
)


class WeightedShrinkageLDATests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rng = np.random.default_rng(20260721)
        n_classes = 6
        p = 14
        rows = []
        labels = []
        for c, n in enumerate((12, 15, 18, 21, 24, 27)):
            latent = rng.normal(size=(n, 5))
            mixing = rng.normal(size=(5, p))
            Xc = latent @ mixing + rng.normal(scale=0.15, size=(n, p)) + c * 0.35
            rows.append(Xc)
            labels.extend([c] * n)
        cls.X = np.vstack(rows)
        cls.y = np.asarray(labels, dtype=int)
        cls.X_test = rng.normal(size=(80, p))

    def test_equal_weights_match_sklearn_for_all_locked_lambdas(self):
        priors = np.full(6, 1 / 6)
        for lam in (0.05, 0.20, 0.50, 0.80, 0.95):
            with self.subTest(shrinkage=lam):
                ours = WeightedShrinkageLDA(lam).fit(
                    self.X, self.y, sample_weight=np.ones(self.y.size)
                )
                ref = LinearDiscriminantAnalysis(
                    solver="lsqr", shrinkage=lam, priors=priors
                ).fit(self.X, self.y)
                np.testing.assert_array_equal(ours.predict(self.X_test), ref.predict(self.X_test))
                np.testing.assert_allclose(ours.means_, ref.means_, atol=1e-12, rtol=1e-12)
                np.testing.assert_allclose(ours.covariance_, ref.covariance_, atol=1e-11, rtol=1e-11)

    def test_weighted_mean_and_covariance_match_manual_formula(self):
        X = np.array(
            [
                [0.0, 0.0],
                [2.0, 1.0],
                [7.0, 5.0],
                [8.0, 9.0],
            ]
        )
        y = np.array([0, 0, 1, 1])
        w = np.array([1.0, 3.0, 2.0, 4.0])
        model = WeightedShrinkageLDA(0.2).fit(X, y, sample_weight=w)
        manual_means = []
        manual_covs = []
        for c in (0, 1):
            mask = y == c
            wc = w[mask]
            Xc = X[mask]
            mu = np.sum(Xc * wc[:, None], axis=0) / wc.sum()
            centered = Xc - mu
            cov = (centered * wc[:, None]).T @ centered / wc.sum()
            manual_means.append(mu)
            manual_covs.append(cov)
        pooled = np.mean(np.stack(manual_covs), axis=0)
        target = np.trace(pooled) / pooled.shape[0]
        expected = 0.8 * pooled + 0.2 * target * np.eye(2)
        np.testing.assert_allclose(model.means_, manual_means)
        np.testing.assert_allclose(model.covariance_, expected)

    def test_weight_scale_invariance(self):
        rng = np.random.default_rng(99)
        w = rng.uniform(0.1, 4.0, size=self.y.size)
        a = WeightedShrinkageLDA(0.5).fit(self.X, self.y, sample_weight=w)
        b = WeightedShrinkageLDA(0.5).fit(self.X, self.y, sample_weight=37.0 * w)
        np.testing.assert_allclose(a.means_, b.means_, atol=1e-12, rtol=1e-12)
        np.testing.assert_allclose(a.covariance_, b.covariance_, atol=1e-12, rtol=1e-12)
        np.testing.assert_array_equal(a.predict(self.X_test), b.predict(self.X_test))

    def test_uniform_priors_are_explicit(self):
        model = WeightedShrinkageLDA(0.8).fit(self.X, self.y)
        np.testing.assert_allclose(model.priors_, np.full(6, 1 / 6))

    def test_exact_collinearity_and_p_greater_than_n_are_stable(self):
        rng = np.random.default_rng(7)
        base = rng.normal(size=(36, 8))
        X = np.column_stack([base, base[:, :5], base[:, [0]], 2.0 * base[:, [1]]])
        X = np.column_stack([X, rng.normal(scale=0.01, size=(36, 30))])
        y = np.repeat(np.arange(6), 6)
        model = WeightedShrinkageLDA(0.05).fit(X, y)
        scores = model.decision_function(X)
        self.assertTrue(np.isfinite(scores).all())
        self.assertEqual(model.predict(X).shape, (36,))

    def test_clone_pickle_joblib_and_repeatability(self):
        original = WeightedShrinkageLDA(0.2)
        cloned = clone(original)
        a = original.fit(self.X, self.y)
        b = cloned.fit(self.X, self.y)
        np.testing.assert_array_equal(a.predict(self.X_test), b.predict(self.X_test))
        with tempfile.TemporaryDirectory(prefix="track-a-lda-") as temp_dir:
            joblib_path = f"{temp_dir}/lda.joblib"
            joblib.dump(a, joblib_path)
            for restored in (pickle.loads(pickle.dumps(a)), joblib.load(joblib_path)):
                np.testing.assert_array_equal(a.predict(self.X_test), restored.predict(self.X_test))

    def test_invalid_weights_fail_closed(self):
        for weights in (
            np.ones(self.y.size - 1),
            np.r_[np.ones(self.y.size - 1), 0.0],
            np.r_[np.ones(self.y.size - 1), -1.0],
            np.r_[np.ones(self.y.size - 1), np.nan],
        ):
            with self.subTest(last=weights[-1]):
                with self.assertRaises(ValueError):
                    WeightedShrinkageLDA(0.2).fit(self.X, self.y, sample_weight=weights)


class MetricsAndStatisticsTests(unittest.TestCase):
    def test_participant_first_pooling_trap(self):
        y = np.tile(np.arange(6), 2)
        pred = np.r_[np.arange(6), np.zeros(6, dtype=int)]
        participants = np.array(["A"] * 6 + ["B"] * 6)
        primary, per = participant_first_macro_f1(y, pred, participants)
        self.assertAlmostEqual(per["A"], 1.0)
        self.assertAlmostEqual(per["B"], 1.0 / 21.0)
        self.assertAlmostEqual(primary, 11.0 / 21.0)
        pooled = f1_score(y, pred, labels=LABELS, average="macro", zero_division=0)
        self.assertAlmostEqual(pooled, 17.0 / 27.0)
        self.assertNotAlmostEqual(primary, pooled)

    def test_fixed_six_includes_missing_class_as_zero(self):
        y = np.arange(5)
        pred = np.arange(5)
        metrics = fixed_six_metrics(y, pred)
        self.assertAlmostEqual(metrics["fixed6_macro_f1"], 5 / 6)
        self.assertEqual(metrics["support"][5], 0)

    def test_sign_flip_fixtures_and_invariances(self):
        d = np.ones(13)
        result = exhaustive_sign_flip(d)
        self.assertEqual(result["total_assignments"], 8192)
        self.assertEqual(result["extreme_assignments"], 2)
        self.assertAlmostEqual(result["p_value"], 2 / 8192)
        self.assertAlmostEqual(exhaustive_sign_flip([1, 1, 0])["p_value"], 0.5)
        base = exhaustive_sign_flip([0.2, -0.1, 0.4, 0.3])["p_value"]
        self.assertEqual(base, exhaustive_sign_flip([-0.2, 0.1, -0.4, -0.3])["p_value"])
        self.assertEqual(base, exhaustive_sign_flip([4.0, -2.0, 8.0, 6.0])["p_value"])
        self.assertEqual(base, exhaustive_sign_flip([0.4, 0.2, 0.3, -0.1])["p_value"])

    def test_holm_fixture(self):
        np.testing.assert_allclose(holm_adjust([0.01, 0.03, 0.04]), [0.03, 0.06, 0.06])
        np.testing.assert_allclose(holm_adjust([0.01, 0.04]), [0.02, 0.04])

    def test_participant_normalized_confusion_is_not_pooled(self):
        y = np.zeros(101, dtype=int)
        pred = np.r_[np.zeros(100, dtype=int), np.ones(1, dtype=int)]
        participants = np.array(["A"] * 100 + ["B"])
        # Add one observation in classes 1..5 for both participants so the fixed-row gate holds.
        y = np.r_[y, np.tile(np.arange(1, 6), 2)]
        pred = np.r_[pred, np.tile(np.arange(1, 6), 2)]
        participants = np.r_[participants, ["A"] * 5, ["B"] * 5]
        mean_cm, per = participant_normalized_confusion(y, pred, participants)
        self.assertAlmostEqual(mean_cm[0, 0], 0.5)
        self.assertAlmostEqual(per["A"][0, 0], 1.0)
        self.assertAlmostEqual(per["B"][0, 0], 0.0)
        self.assertTrue(np.allclose(mean_cm.sum(axis=1), 1.0))
        self.assertNotAlmostEqual(mean_cm[0, 0], 100 / 101)

    def test_bootstrap_constant_and_index_hash(self):
        indices = make_bootstrap_indices(13, repetitions=10000, seed=20260721)
        low, high = bootstrap_mean_ci(np.full(13, 0.1), indices)
        self.assertAlmostEqual(low, 0.1)
        self.assertAlmostEqual(high, 0.1)
        digest = hashlib.sha256(indices.tobytes()).hexdigest()
        self.assertEqual(digest, hashlib.sha256(make_bootstrap_indices(13).tobytes()).hexdigest())


class WeightsAndRegistryTests(unittest.TestCase):
    def test_preprocessing_is_fit_on_training_rows_only(self):
        X_train = np.array(
            [
                [1.0, 0.0, 2.0],
                [2.0, 1.0, 3.0],
                [3.0, 0.0, 4.0],
                [4.0, 1.0, 5.0],
                [5.0, 0.0, 6.0],
                [6.0, 1.0, 7.0],
            ]
        )
        y_train = np.arange(6)
        procedure = FittedProcedure("logistic", {"family": "l2", "C": 0.1}, 20260721)
        procedure.fit(X_train, y_train, np.ones(6))
        imputer_before = procedure.imputer.statistics_.copy()
        scaler_before = procedure.scaler.mean_.copy()
        X_test = np.array([[1e9, -1e9, np.nan]])
        procedure.predict(X_test)
        np.testing.assert_array_equal(procedure.imputer.statistics_, imputer_before)
        np.testing.assert_array_equal(procedure.scaler.mean_, scaler_before)

    def test_subject_only_has_equal_total_weight_per_subject_and_mean_one(self):
        subjects = np.array(["A"] * 2 + ["B"] * 4 + ["C"] * 3)
        y = np.arange(subjects.size) % 6
        w = compute_training_weights(subjects, y, "subject_only")
        self.assertAlmostEqual(float(w.mean()), 1.0)
        totals = [w[subjects == s].sum() for s in sorted(set(subjects))]
        np.testing.assert_allclose(totals, np.full(3, totals[0]))

    def test_unweighted_is_one(self):
        subjects = np.array(["A", "A", "B"])
        y = np.array([0, 1, 0])
        np.testing.assert_array_equal(compute_training_weights(subjects, y, "unweighted"), 1.0)

    def test_capped_subject_class_uses_rowwise_raw_median_then_normalizes(self):
        subjects = np.array(["A"] * 7 + ["B"] * 3)
        y = np.array([0, 0, 0, 0, 0, 0, 1, 0, 0, 1])
        raw = np.array([1 / 6] * 6 + [1.0] + [1 / 2] * 2 + [1.0])
        expected = np.minimum(raw, 3 * np.median(raw))
        expected /= expected.mean()
        actual = compute_training_weights(subjects, y, "capped_subject_class")
        np.testing.assert_allclose(actual, expected)
        self.assertAlmostEqual(float(actual.mean()), 1.0)

    def test_registry_counts_and_lda_order(self):
        registry = build_model_registry()
        validate_registry_counts(registry)
        self.assertEqual(
            [c.params["shrinkage"] for c in registry["weighted_shrinkage_lda"]],
            [0.95, 0.80, 0.50, 0.20, 0.05],
        )
        elastic = [c for c in registry["logistic"] if c.params["family"] == "elastic_net"]
        self.assertEqual(
            [c.params["l1_ratio"] for c in elastic],
            [0.25, 0.50, 0.75] * 3,
        )
        self.assertTrue(all("gamma" not in c.params for c in registry["rbf_svm"]))

    def test_rbf_gamma_uses_training_fold_post_variance_feature_count(self):
        X = np.column_stack(
            [
                np.linspace(-1.0, 1.0, 18),
                np.tile([0.0, 1.0, 2.0], 6),
                np.ones(18),
            ]
        )
        y = np.tile(np.arange(6), 3)
        procedure = FittedProcedure(
            "rbf_svm", {"C": 1.0, "gamma_multiplier": 4.0}, 20260721
        ).fit(X, y, np.ones(len(y)))
        self.assertEqual(procedure.n_features_after_variance_, 2)
        self.assertEqual(procedure.estimator.gamma, 2.0)

    def test_config_selection_fails_closed_on_incomplete_matrix(self):
        configs = build_model_registry()["linear_svm"]
        with self.assertRaises(RuntimeError):
            select_config(configs, {configs[0].config_id: 0.8})

    def test_warning_is_preserved_when_fit_then_fails(self):
        def warning_then_failure(_self, _X, _y, _weights):
            warnings.warn("synthetic warning before failure", RuntimeWarning)
            raise RuntimeError("synthetic fit failure")

        X = np.arange(36, dtype=float).reshape(12, 3)
        y = np.tile(np.arange(6), 2)
        subjects = np.repeat(["A", "B"], 6)
        with mock.patch.object(FittedProcedure, "fit", new=warning_then_failure):
            result = fit_predict_once(
                "logistic",
                {"family": "l2", "C": 0.1},
                20260721,
                X,
                y,
                subjects,
                X[:2],
                "subject_only",
            )
        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["failure_type"], "RuntimeError")
        self.assertEqual(result["warnings"][0]["category"], "RuntimeWarning")
        self.assertGreaterEqual(result["fit_seconds"], 0.0)

    def test_fixed_seed_random_estimators_repeat_exactly(self):
        rng = np.random.default_rng(20260721)
        X = rng.normal(size=(72, 9))
        y = np.tile(np.arange(6), 12)
        X += y[:, None] * 0.1
        subjects = np.repeat(["A", "B", "C", "D", "E", "F"], 12)
        cases = [
            ("logistic", {"family": "elastic_net", "C": 0.3, "l1_ratio": 0.5}),
            (
                "random_forest",
                {
                    "n_estimators": 20,
                    "max_depth": 4,
                    "min_samples_leaf": 2,
                    "max_features": "sqrt",
                },
            ),
            (
                "hist_gradient_boosting",
                {
                    "learning_rate": 0.1,
                    "max_iter": 30,
                    "max_depth": 2,
                    "l2_regularization": 1.0,
                    "min_samples_leaf": 5,
                    "early_stopping": False,
                },
            ),
        ]
        for model, params in cases:
            with self.subTest(model=model):
                a = fit_predict_once(
                    model, params, 20260721, X, y, subjects, X, "subject_only"
                )
                b = fit_predict_once(
                    model, params, 20260721, X, y, subjects, X, "subject_only"
                )
                self.assertNotEqual(a["status"], "failed")
                self.assertNotEqual(b["status"], "failed")
                np.testing.assert_array_equal(a["prediction"], b["prediction"])

    def test_practical_tie_selects_simplest(self):
        configs = build_model_registry()["linear_svm"]
        scores = {c.config_id: 0.8 for c in configs}
        scores[configs[-1].config_id] = 0.809
        selected, details = select_config(configs, scores)
        self.assertEqual(selected.config_id, configs[0].config_id)
        self.assertGreaterEqual(len(details["practically_tied_config_ids"]), 2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
