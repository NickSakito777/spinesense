from __future__ import annotations

"""Strict per-trial IMU-to-anatomy placement-map resolver.

Firmware ``IMU0`` ... ``IMU4`` identifiers describe physical interfaces, not
anatomy.  Production analysis must therefore resolve a complete mapping for the
specific capture instead of inheriting a global layout preset.

The registry schema is::

    {
      "schema_version": 1,
      "mapping_version": "mapping_v1",
      "trials": {
        "T90_P90": {
          "role_to_imu": {
            "sacrum": "IMU1", "lower": "IMU2", "mid": "IMU3",
            "upper": "IMU4", "sternum": "IMU0"
          },
          "status": "inferred_high",
          "confidence": "high",
          "evidence": ["..."],
          "source_refs": ["..."],
          "raw_segments": ["T02-2.log"]
        }
      }
    }

``raw_segments`` may instead be a ``segment-id -> path`` mapping.  A structured
segment value may use one of ``raw``, ``path`` or ``source`` for its path.
"""

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
from types import MappingProxyType
from typing import Any, Iterable, Mapping


ROLES = ("sacrum", "lower", "mid", "upper", "sternum")
IMU_IDS = ("IMU0", "IMU1", "IMU2", "IMU3", "IMU4")

REGISTRY_SCHEMA_VERSION = 1
KNOWN_STATUSES = frozenset({"confirmed", "inferred_high", "mixed", "unconfirmed"})
PRODUCTION_STATUSES = frozenset({"confirmed", "inferred_high"})

DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent / "config" / "placement_maps_v1.json"


class PlacementMapError(ValueError):
    """Base class for deterministic registry, lookup and policy failures."""


class PlacementMapConfigError(PlacementMapError):
    """The placement registry is missing, malformed or internally ambiguous."""


class PlacementMapLookupError(PlacementMapError):
    """No unique registry entry matches the requested trial/raw segment."""


class PlacementMapStatusError(PlacementMapError):
    """The resolved entry exists but is not eligible under the requested policy."""


def _canonical_json(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))


def _portable_basename(value: str | Path) -> str:
    """Return the final component for either POSIX or Windows-form paths."""
    return Path(str(value).replace("\\", "/")).name


def _normalise_trial_id(value: str) -> str:
    trial_id = str(value).strip().upper()
    if not trial_id:
        raise PlacementMapLookupError("trial_id must not be empty")
    return trial_id


def _normalise_string_list(value: Any, field: str, trial_id: str) -> tuple[str, ...]:
    if value is None:
        return ()
    if isinstance(value, str):
        values = [value]
    elif isinstance(value, list):
        values = value
    else:
        raise PlacementMapConfigError(f"{trial_id}.{field} must be a string or list of strings")
    out: list[str] = []
    for item in values:
        if not isinstance(item, str) or not item.strip():
            raise PlacementMapConfigError(f"{trial_id}.{field} contains a non-string or empty value")
        out.append(item.strip())
    return tuple(out)


def _segment_path(value: Any, field: str) -> str:
    if isinstance(value, str):
        raw = value
    elif isinstance(value, Mapping):
        present = [key for key in ("raw", "path", "source") if value.get(key)]
        if len(present) != 1:
            raise PlacementMapConfigError(
                f"{field} structured segment must contain exactly one of raw/path/source"
            )
        raw = value[present[0]]
    else:
        raise PlacementMapConfigError(f"{field} must be a path string or path-bearing object")
    if not isinstance(raw, str) or not raw.strip():
        raise PlacementMapConfigError(f"{field} path must be a non-empty string")
    return raw.strip()


def _normalise_raw_segments(value: Any, trial_id: str) -> tuple[str, ...]:
    if value is None:
        return ()
    paths: list[str] = []
    if isinstance(value, list):
        for index, item in enumerate(value):
            paths.append(_segment_path(item, f"{trial_id}.raw_segments[{index}]"))
    elif isinstance(value, Mapping):
        for segment_id, item in value.items():
            if not isinstance(segment_id, str) or not segment_id.strip():
                raise PlacementMapConfigError(f"{trial_id}.raw_segments has an invalid segment id")
            paths.append(_segment_path(item, f"{trial_id}.raw_segments.{segment_id}"))
    else:
        raise PlacementMapConfigError(f"{trial_id}.raw_segments must be a list or object")
    if len(paths) != len(set(paths)):
        raise PlacementMapConfigError(f"{trial_id}.raw_segments contains duplicate paths")
    return tuple(sorted(paths))


def _validate_role_map(value: Any, trial_id: str) -> tuple[tuple[str, str], ...]:
    if not isinstance(value, Mapping):
        raise PlacementMapConfigError(f"{trial_id}.role_to_imu must be an object")

    keys = {str(key) for key in value}
    expected = set(ROLES)
    missing = sorted(expected - keys)
    extra = sorted(keys - expected)
    if missing or extra:
        raise PlacementMapConfigError(
            f"{trial_id}.role_to_imu must contain exactly {list(ROLES)}; "
            f"missing={missing}, extra={extra}"
        )

    normalised: dict[str, str] = {}
    for role in ROLES:
        imu = value[role]
        if not isinstance(imu, str):
            raise PlacementMapConfigError(f"{trial_id}.role_to_imu.{role} must be a string")
        imu = imu.strip().upper()
        if imu not in IMU_IDS:
            raise PlacementMapConfigError(
                f"{trial_id}.role_to_imu.{role}={imu!r}; expected one of {list(IMU_IDS)}"
            )
        normalised[role] = imu

    values = tuple(normalised[role] for role in ROLES)
    if len(set(values)) != len(values):
        duplicates = sorted({imu for imu in values if values.count(imu) > 1})
        raise PlacementMapConfigError(
            f"{trial_id}.role_to_imu is not one-to-one; duplicate IMU ids: {duplicates}"
        )
    if set(values) != set(IMU_IDS):
        raise PlacementMapConfigError(
            f"{trial_id}.role_to_imu must be a bijection over {list(IMU_IDS)}"
        )
    return tuple((role, normalised[role]) for role in ROLES)


@dataclass(frozen=True)
class ResolvedPlacement:
    """Immutable, validated mapping plus its provenance and canonical digest."""

    trial_id: str
    mapping_version: str
    status: str
    config_path: str
    canonical_sha256: str
    raw_segments: tuple[str, ...]
    _role_items: tuple[tuple[str, str], ...]
    _provenance_json: str

    @property
    def role_to_imu(self) -> Mapping[str, str]:
        """Read-only anatomical-role -> firmware-ID mapping."""
        return MappingProxyType(dict(self._role_items))

    @property
    def provenance(self) -> Mapping[str, Any]:
        """Read-only top-level provenance metadata copied from the registry entry."""
        return MappingProxyType(json.loads(self._provenance_json))

    @property
    def placement_map_source(self) -> str:
        return f"registry:{self.config_path}#{self.trial_id}"

    def role_overrides(self) -> dict[str, str]:
        """Keyword overrides accepted by the existing ``five_imu_fusion.make_args``."""
        return dict(self._role_items)

    def provenance_kwargs(self) -> dict[str, str]:
        """Provenance kwargs for the provenance-aware fusion API.

        The fusion parser owns these field names.  Keeping them separate from
        :meth:`role_overrides` also lets legacy callers pass only the currently
        supported anatomical role overrides during a staged migration.
        """
        return {
            "trial_id": self.trial_id,
            "placement_map_source": self.placement_map_source,
            "mapping_version": self.mapping_version,
            "mapping_status": self.status,
            "mapping_sha256": self.canonical_sha256,
        }

    def fusion_kwargs(self) -> dict[str, str]:
        """Role overrides plus provenance for ``fiv.make_args(**kwargs)``."""
        return {
            "layout_preset": "trial5",
            **self.role_overrides(),
            **self.provenance_kwargs(),
        }

    def provenance_record(self) -> dict[str, Any]:
        """Serializable record suitable for QC, block and dataset manifests."""
        return {
            "trial_id": self.trial_id,
            "placement_map": self.role_overrides(),
            "placement_map_source": self.placement_map_source,
            "mapping_version": self.mapping_version,
            "mapping_status": self.status,
            "mapping_sha256": self.canonical_sha256,
            "raw_segments": list(self.raw_segments),
            "provenance": dict(self.provenance),
        }


def _build_entry(
    trial_id: str,
    raw: Any,
    *,
    mapping_version: str,
    config_path: Path,
) -> ResolvedPlacement:
    if not isinstance(raw, Mapping):
        raise PlacementMapConfigError(f"trials.{trial_id} must be an object")
    role_items = _validate_role_map(raw.get("role_to_imu"), trial_id)

    status = raw.get("status")
    if not isinstance(status, str) or status not in KNOWN_STATUSES:
        raise PlacementMapConfigError(
            f"{trial_id}.status={status!r}; expected one of {sorted(KNOWN_STATUSES)}"
        )

    raw_segments = _normalise_raw_segments(raw.get("raw_segments"), trial_id)
    if not raw_segments:
        raise PlacementMapConfigError(
            f"{trial_id}.raw_segments must name at least one raw capture/segment"
        )
    evidence = _normalise_string_list(raw.get("evidence"), "evidence", trial_id)
    source_refs = _normalise_string_list(raw.get("source_refs"), "source_refs", trial_id)
    confidence = raw.get("confidence")
    if confidence is not None and not isinstance(confidence, (str, int, float)):
        raise PlacementMapConfigError(f"{trial_id}.confidence must be a string or number")
    notes = raw.get("notes")
    if notes is not None and not isinstance(notes, str):
        raise PlacementMapConfigError(f"{trial_id}.notes must be a string")

    provenance = {
        "confidence": confidence,
        "evidence": list(evidence),
        "source_refs": list(source_refs),
        "notes": notes,
    }
    canonical = {
        "schema_version": REGISTRY_SCHEMA_VERSION,
        "mapping_version": mapping_version,
        "trial_id": trial_id,
        "role_to_imu": dict(role_items),
        "status": status,
        "raw_segments": list(raw_segments),
        "provenance": provenance,
    }
    digest = hashlib.sha256(_canonical_json(canonical).encode("utf-8")).hexdigest()
    return ResolvedPlacement(
        trial_id=trial_id,
        mapping_version=mapping_version,
        status=status,
        config_path=str(config_path.resolve()),
        canonical_sha256=digest,
        raw_segments=raw_segments,
        _role_items=role_items,
        _provenance_json=_canonical_json(provenance),
    )


def load_placement_registry(
    config_path: str | Path = DEFAULT_CONFIG_PATH,
) -> Mapping[str, ResolvedPlacement]:
    """Load and fully validate a placement registry.

    Validation is eager: a broken entry or ambiguous raw basename invalidates the
    registry even when another trial was requested.  This prevents a partially
    corrupt registry from producing apparently valid canonical outputs.
    """
    path = Path(config_path)
    if not path.is_file():
        raise PlacementMapConfigError(f"placement-map registry does not exist: {path}")
    try:
        root = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise PlacementMapConfigError(f"placement-map registry is not valid JSON: {path}: {exc}") from exc
    if not isinstance(root, Mapping):
        raise PlacementMapConfigError("placement-map registry root must be an object")
    if root.get("schema_version") != REGISTRY_SCHEMA_VERSION:
        raise PlacementMapConfigError(
            f"schema_version must be {REGISTRY_SCHEMA_VERSION}, got {root.get('schema_version')!r}"
        )
    mapping_version = root.get("mapping_version")
    if not isinstance(mapping_version, str) or not mapping_version.strip():
        raise PlacementMapConfigError("mapping_version must be a non-empty string")
    mapping_version = mapping_version.strip()
    trials = root.get("trials")
    if not isinstance(trials, Mapping) or not trials:
        raise PlacementMapConfigError("trials must be a non-empty object")

    entries: dict[str, ResolvedPlacement] = {}
    for unnormalised_id, raw in trials.items():
        if not isinstance(unnormalised_id, str):
            raise PlacementMapConfigError("trial ids must be strings")
        try:
            trial_id = _normalise_trial_id(unnormalised_id)
        except PlacementMapLookupError as exc:
            raise PlacementMapConfigError("trial ids must not be empty") from exc
        if trial_id in entries:
            raise PlacementMapConfigError(f"duplicate trial id after normalization: {trial_id}")
        entries[trial_id] = _build_entry(
            trial_id,
            raw,
            mapping_version=mapping_version,
            config_path=path,
        )

    raw_owner: dict[str, str] = {}
    for trial_id, entry in entries.items():
        for raw_segment in entry.raw_segments:
            basename = _portable_basename(raw_segment)
            if not basename:
                raise PlacementMapConfigError(f"{trial_id} has an invalid raw segment: {raw_segment!r}")
            owner = raw_owner.get(basename)
            if owner is not None and owner != trial_id:
                raise PlacementMapConfigError(
                    f"raw segment basename {basename!r} is ambiguous between {owner} and {trial_id}"
                )
            raw_owner[basename] = trial_id
    return MappingProxyType(entries)


def _allowed_status_set(allowed_statuses: Iterable[str] | None) -> frozenset[str]:
    if allowed_statuses is None:
        return KNOWN_STATUSES
    allowed = frozenset(str(status) for status in allowed_statuses)
    invalid = sorted(allowed - KNOWN_STATUSES)
    if invalid:
        raise PlacementMapStatusError(
            f"allowed_statuses contains unknown values {invalid}; known={sorted(KNOWN_STATUSES)}"
        )
    if not allowed:
        raise PlacementMapStatusError("allowed_statuses must not be empty")
    return allowed


def resolve_placement(
    *,
    trial_id: str | None = None,
    raw_path: str | Path | None = None,
    config_path: str | Path = DEFAULT_CONFIG_PATH,
    allowed_statuses: Iterable[str] | None = PRODUCTION_STATUSES,
) -> ResolvedPlacement:
    """Resolve one trial by trial id, raw-segment basename, or both.

    By default only ``confirmed`` and ``inferred_high`` entries are eligible for
    production.  Pass ``allowed_statuses=None`` for a read-only audit that must
    inspect mixed/unconfirmed entries.  When both lookup keys are supplied they
    must resolve to the same trial.
    """
    if trial_id is None and raw_path is None:
        raise PlacementMapLookupError("provide trial_id and/or raw_path")
    registry = load_placement_registry(config_path)

    by_trial: ResolvedPlacement | None = None
    if trial_id is not None:
        key = _normalise_trial_id(trial_id)
        by_trial = registry.get(key)
        if by_trial is None:
            raise PlacementMapLookupError(f"trial {key!r} is not present in {Path(config_path)}")

    by_raw: ResolvedPlacement | None = None
    if raw_path is not None:
        basename = _portable_basename(raw_path)
        if not basename:
            raise PlacementMapLookupError("raw_path must have a filename")
        matches = [
            entry
            for entry in registry.values()
            if basename in {_portable_basename(p) for p in entry.raw_segments}
        ]
        if not matches:
            raise PlacementMapLookupError(
                f"raw segment {basename!r} is not present in {Path(config_path)}"
            )
        if len(matches) != 1:
            raise PlacementMapLookupError(
                f"raw segment {basename!r} resolves ambiguously to {[entry.trial_id for entry in matches]}"
            )
        by_raw = matches[0]

    if by_trial is not None and by_raw is not None and by_trial.trial_id != by_raw.trial_id:
        raise PlacementMapLookupError(
            f"trial_id resolves to {by_trial.trial_id}, but raw_path resolves to {by_raw.trial_id}"
        )
    resolved = by_trial or by_raw
    assert resolved is not None

    allowed = _allowed_status_set(allowed_statuses)
    if resolved.status not in allowed:
        raise PlacementMapStatusError(
            f"{resolved.trial_id} mapping status {resolved.status!r} is not eligible; "
            f"allowed={sorted(allowed)}"
        )
    return resolved
