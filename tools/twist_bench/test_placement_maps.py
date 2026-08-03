from __future__ import annotations

from dataclasses import FrozenInstanceError
import json
from pathlib import Path
import tempfile
import unittest

import placement_maps as pm


REVERSE_MAP = {
    "sacrum": "IMU1",
    "lower": "IMU2",
    "mid": "IMU3",
    "upper": "IMU4",
    "sternum": "IMU0",
}

T91_MAP = {
    "sacrum": "IMU2",
    "lower": "IMU1",
    "mid": "IMU3",
    "upper": "IMU4",
    "sternum": "IMU0",
}


def registry_payload() -> dict:
    return {
        "schema_version": 1,
        "mapping_version": "mapping_v1",
        "trials": {
            "T90_P90": {
                "role_to_imu": dict(REVERSE_MAP),
                "status": "confirmed",
                "confidence": "high",
                "evidence": ["field record", "IMU-MoCap permutation"],
                "source_refs": ["photos/T90.jpg"],
                "raw_segments": ["data/sessions/T90-2.log"],
            },
            "T91_P91": {
                "role_to_imu": dict(T91_MAP),
                "status": "inferred_high",
                "confidence": 0.91,
                "evidence": ["IMU-MoCap permutation"],
                "source_refs": [],
                "raw_segments": {
                    "clean": {"path": "data_clean/T91_P91/segmented.csv"},
                    "original": "data/sessions/T91.log",
                },
            },
        },
    }


class PlacementMapTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.config = Path(self.tmp.name) / "placement_maps_v1.json"

    def write(self, payload: dict) -> Path:
        self.config.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        return self.config

    def assert_config_error(self, payload: dict) -> None:
        self.write(payload)
        with self.assertRaises(pm.PlacementMapConfigError):
            pm.load_placement_registry(self.config)

    def test_missing_registry_and_trial_fail_loudly(self) -> None:
        with self.assertRaises(pm.PlacementMapConfigError):
            pm.resolve_placement(trial_id="T90_P90", config_path=self.config)
        self.write(registry_payload())
        with self.assertRaises(pm.PlacementMapLookupError):
            pm.resolve_placement(trial_id="T99_P99", config_path=self.config)

    def test_partial_map_rejected(self) -> None:
        payload = registry_payload()
        del payload["trials"]["T90_P90"]["role_to_imu"]["sternum"]
        self.assert_config_error(payload)

    def test_missing_raw_segments_rejected(self) -> None:
        payload = registry_payload()
        del payload["trials"]["T90_P90"]["raw_segments"]
        self.assert_config_error(payload)

    def test_duplicate_imu_rejected(self) -> None:
        payload = registry_payload()
        payload["trials"]["T90_P90"]["role_to_imu"]["upper"] = "IMU3"
        self.assert_config_error(payload)

    def test_invalid_imu_rejected(self) -> None:
        payload = registry_payload()
        payload["trials"]["T90_P90"]["role_to_imu"]["upper"] = "IMU9"
        self.assert_config_error(payload)

    def test_invalid_and_ineligible_statuses(self) -> None:
        payload = registry_payload()
        payload["trials"]["T90_P90"]["status"] = "probably"
        self.assert_config_error(payload)

        payload = registry_payload()
        payload["trials"]["T90_P90"]["status"] = "mixed"
        self.write(payload)
        with self.assertRaises(pm.PlacementMapStatusError):
            pm.resolve_placement(trial_id="T90_P90", config_path=self.config)
        audit = pm.resolve_placement(
            trial_id="T90_P90", config_path=self.config, allowed_statuses=None
        )
        self.assertEqual(audit.status, "mixed")

    def test_two_trials_resolve_to_different_maps(self) -> None:
        self.write(registry_payload())
        t02 = pm.resolve_placement(trial_id="t90_p90", config_path=self.config)
        t03 = pm.resolve_placement(trial_id="T91_P91", config_path=self.config)
        self.assertEqual(t02.role_to_imu["sacrum"], "IMU1")
        self.assertEqual(t03.role_to_imu["sacrum"], "IMU2")
        self.assertNotEqual(t02.canonical_sha256, t03.canonical_sha256)

    def test_raw_segment_lookup_and_trial_cross_check(self) -> None:
        self.write(registry_payload())
        t03 = pm.resolve_placement(
            raw_path="/arbitrary/host/path/segmented.csv", config_path=self.config
        )
        self.assertEqual(t03.trial_id, "T91_P91")
        windows = pm.resolve_placement(
            raw_path=r"C:\\capture\\segmented.csv", config_path=self.config
        )
        self.assertEqual(windows.trial_id, "T91_P91")
        same = pm.resolve_placement(
            trial_id="T91_P91", raw_path="T91.log", config_path=self.config
        )
        self.assertEqual(same.trial_id, "T91_P91")
        with self.assertRaises(pm.PlacementMapLookupError):
            pm.resolve_placement(
                trial_id="T90_P90", raw_path="T91.log", config_path=self.config
            )

    def test_hash_is_stable_across_json_and_role_key_order(self) -> None:
        first = registry_payload()
        self.write(first)
        h1 = pm.resolve_placement(trial_id="T90_P90", config_path=self.config).canonical_sha256

        second = registry_payload()
        role_map = second["trials"]["T90_P90"]["role_to_imu"]
        second["trials"]["T90_P90"]["role_to_imu"] = {
            key: role_map[key] for key in reversed(tuple(role_map))
        }
        # Reorder trials and compact the JSON to prove formatting/root order are irrelevant.
        second["trials"] = dict(reversed(tuple(second["trials"].items())))
        self.config.write_text(json.dumps(second, separators=(",", ":")), encoding="utf-8")
        h2 = pm.resolve_placement(trial_id="T90_P90", config_path=self.config).canonical_sha256
        self.assertEqual(h1, h2)

    def test_result_is_immutable_and_exposes_fusion_provenance(self) -> None:
        self.write(registry_payload())
        placement = pm.resolve_placement(trial_id="T90_P90", config_path=self.config)
        with self.assertRaises(FrozenInstanceError):
            placement.status = "unconfirmed"  # type: ignore[misc]
        with self.assertRaises(TypeError):
            placement.role_to_imu["sacrum"] = "IMU4"  # type: ignore[index]

        kwargs = placement.fusion_kwargs()
        self.assertEqual(kwargs["layout_preset"], "trial5")
        self.assertEqual(kwargs["sacrum"], "IMU1")
        self.assertEqual(kwargs["trial_id"], "T90_P90")
        self.assertEqual(kwargs["mapping_version"], "mapping_v1")
        self.assertEqual(kwargs["mapping_sha256"], placement.canonical_sha256)
        record = placement.provenance_record()
        self.assertEqual(record["placement_map"], REVERSE_MAP)
        self.assertEqual(record["mapping_status"], "confirmed")


if __name__ == "__main__":
    unittest.main()
