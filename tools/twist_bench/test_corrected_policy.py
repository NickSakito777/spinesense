from __future__ import annotations

from types import SimpleNamespace
import unittest

import numpy as np

from coverage_gate import assess_scoring_coverage
from corrected_validation_rebuild import corrected_quality, normalize_metric, score_uniform_zero_lag


class CorrectedPolicyTests(unittest.TestCase):
    def test_in_sample_calibrated_rmse_is_not_relabeled_heldout(self) -> None:
        metric = normalize_metric({"rmse_after_gain_deg": 1.2, "pooled_r": 0.9})
        self.assertEqual(metric["calibrated_rmse_deg"], 1.2)
        self.assertNotIn("heldout_rmse_deg", metric)

    def test_single_rep_is_invalid_without_true_heldout(self) -> None:
        quality, reasons = corrected_quality(
            {"pooled_r": 0.99, "calibrated_rmse_deg": 0.2}, 1, 1
        )
        self.assertEqual(quality, "invalid_insufficient_repetitions")
        self.assertIn("missing_true_heldout", reasons)

    def test_uniform_quality_and_gate(self) -> None:
        clean, _ = corrected_quality(
            {"pooled_r": 0.9, "heldout_rmse_deg": 2.0, "heldout_acc": 0.8}, 18, 20
        )
        low, _ = corrected_quality(
            {"pooled_r": 0.75, "heldout_rmse_deg": 4.0, "heldout_acc": 0.6}, 8, 10
        )
        invalid, _ = corrected_quality(
            {"pooled_r": 0.95, "heldout_rmse_deg": 1.0, "heldout_acc": 0.9}, 3, 20
        )
        self.assertEqual(clean, "clean")
        self.assertEqual(low, "low_conf")
        self.assertEqual(invalid, "invalid_quality_gate")

    def test_movement_and_neutral_both_require_timestamp_support(self) -> None:
        complete = np.arange(0.0, 5.0, 0.01)
        accepted, qc = assess_scoring_coverage(complete, 1.0, 0.0, [(2.0, 3.0)], np)
        self.assertEqual(accepted, [(2.0, 3.0)])
        self.assertTrue(qc[0]["accepted"])

        missing_neutral = complete[(complete < 0.8) | (complete > 1.8)]
        accepted, qc = assess_scoring_coverage(
            missing_neutral, 1.0, 0.0, [(2.0, 3.0)], np
        )
        self.assertEqual(accepted, [])
        self.assertFalse(qc[0]["neutral"]["accepted"])

    def test_coverage_rejects_partial_support_and_long_internal_gap(self) -> None:
        complete = np.arange(0.0, 5.0, 0.01)
        partial = complete[(complete < 2.45) | (complete > 2.55)]
        accepted, qc = assess_scoring_coverage(partial, 1.0, 0.0, [(2.0, 3.0)], np)
        self.assertEqual(accepted, [])
        self.assertIn(
            "max_gap_gt_2x_nearest_tolerance",
            qc[0]["movement"]["exclusion_reasons"],
        )

    def test_uniform_score_produces_true_loro_only_with_multiple_bouts(self) -> None:
        ti = np.arange(0.0, 8.0, 0.01)
        signal = np.sin(ti)
        res = SimpleNamespace(t_s=ti)
        factory = lambda _lo: signal.copy()
        one = score_uniform_zero_lag(
            res=res, a=1.0, b=0.0, tm=ti, signal=signal, bouts=[(2.0, 3.0)],
            series_factory=factory, abs_mocap=False, np=np,
        )
        self.assertNotIn("heldout_rmse_deg", one)
        two = score_uniform_zero_lag(
            res=res, a=1.0, b=0.0, tm=ti, signal=signal,
            bouts=[(2.0, 3.0), (4.0, 5.0)], series_factory=factory,
            abs_mocap=False, np=np,
        )
        self.assertIn("heldout_rmse_deg", two)
        self.assertIn("per_bout_heldout_rmse_deg", two)


if __name__ == "__main__":
    unittest.main()
