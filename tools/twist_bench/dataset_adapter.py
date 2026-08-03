from __future__ import annotations

"""Configuration-driven access to a session dataset.

The analysis scripts in this repository need to answer four questions about a
cohort of recordings:

    where is session S's IMU log / mocap file / block manifest?
    which protocol blocks does it contain, and what movement is each one?
    which of those blocks are trustworthy?
    how do I load and fuse the session?

None of those answers belong in the analysis code. They describe *one* dataset:
its path layout, its session identifiers, the per-session corrections a human
made while cleaning it, and the quality judgements attached to individual
blocks. Hard-coding them turns a general method into a script that runs on
exactly one cohort -- and bakes the cohort's identifiers into the source.

So this module holds the *logic* and reads the *dataset* from a config file.
Point it at your own config and the same analysis code runs on your recordings.

    import dataset_adapter as da
    da.configure("my_dataset.json")
    for bid, block, res, a, b, bouts in da.subject_blocks("01", tm, msig):
        ...

The config used for the dissertation is not published, because it describes
human-subject recordings. ``docs/dataset_config.example.json`` shows the schema
with placeholder values.

Config schema
-------------
    root              base directory for the templates below (default: this file's parent)
    paths.imu         template, ``{s}`` = session id       e.g. "data/sessions/{s}.log"
    paths.mocap       template                             e.g. "data/mocap/{s}.csv"
    paths.manifest    template                             e.g. "data_clean/{s}/block_manifest.json"
    paths.trial_id    template for the placement registry  e.g. "{s}"
    sessions          list of session ids to iterate
    block_overrides   {session: {block_id: {label?, window?, quality?, sig?, sign?}}}
    sign_overrides    {"session:block_id": int}
    sflp_fallback     {session: "bend_quat_finite" | "twist_variance"}
                      sessions whose on-chip SFLP stream is unreliable and should fall
                      back to the VQF filter when the named check fails
"""

import json
from pathlib import Path
from typing import Any, Iterator

import numpy as np

import five_imu_fusion as fiv
import placement_maps as pm
import session_recipe as sr
import signed_diagnostic as sd  # re-exported: callers reach mocap_signed through this module

# Re-exported from session_recipe so analysis scripts have one import to reach both
# the dataset layer and the geometry it needs.
from session_recipe import swing_rotvec_deg, tared_bend_quat  # noqa: F401

EPS = 1e-9

# --- dataset-independent constants ---------------------------------------
# These describe the movement taxonomy, not any particular cohort.

LABELS = {
    "flexion": 0, "extension": 1, "left_bend": 2,
    "right_bend": 3, "left_twist": 4, "right_twist": 5,
}

# Which anatomical mocap channel carries each movement. Derived from the label
# rather than read from the manifest: older manifests omit the per-block signal
# key, and a missing field silently falling back to the wrong channel is worse
# than not consulting the field at all.
SIG_BY_LABEL = {
    "flexion": "flex", "extension": "flex", "left_bend": "lat",
    "right_bend": "lat", "left_twist": "axial", "right_twist": "axial",
}

SIGN_BY_LABEL = {
    "flexion": 1, "extension": -1, "left_bend": -1,
    "right_bend": 1, "left_twist": -1, "right_twist": 1,
}

_DEFAULT_PATHS = {
    "imu": "data/sessions/{s}.log",
    "mocap": "data/mocap/{s}.csv",
    "manifest": "data_clean/{s}/block_manifest.json",
    "trial_id": "{s}",
}


# --- configuration --------------------------------------------------------

class DatasetConfig:
    """One dataset's layout, session list, and per-session corrections."""

    def __init__(self, payload: dict[str, Any], source: Path | None = None):
        self.source = source
        base = payload.get("root")
        self.root = Path(base) if base else (source.parent if source else Path.cwd())
        if not self.root.is_absolute() and source:
            self.root = (source.parent / self.root).resolve()
        self.paths = {**_DEFAULT_PATHS, **payload.get("paths", {})}
        self.sessions: list[str] = [str(s) for s in payload.get("sessions", [])]
        self.block_overrides: dict[str, dict] = payload.get("block_overrides", {})
        self.sflp_fallback: dict[str, str] = payload.get("sflp_fallback", {})
        self.unavailable_sessions: dict[str, str] = payload.get("unavailable_sessions", {})
        raw_signs = payload.get("sign_overrides", {})
        self.sign_overrides: dict[tuple[str, str], int] = {}
        for key, value in raw_signs.items():
            session, _, block = str(key).partition(":")
            if not block:
                raise ValueError(f"sign_overrides key {key!r} must be 'session:block_id'")
            self.sign_overrides[(session, block)] = int(value)

    def path(self, kind: str, session: str) -> Path:
        return self.root / self.paths[kind].format(s=session)

    def trial_id(self, session: str) -> str:
        return self.paths["trial_id"].format(s=session)


_CFG: DatasetConfig | None = None


def configure(config: str | Path | dict[str, Any]) -> DatasetConfig:
    """Point this module at a dataset. Accepts a path to JSON or an inline dict."""
    global _CFG
    if isinstance(config, (str, Path)):
        source = Path(config).resolve()
        if not source.exists():
            raise FileNotFoundError(f"dataset config not found: {source}")
        _CFG = DatasetConfig(json.loads(source.read_text(encoding="utf-8")), source)
    else:
        _CFG = DatasetConfig(dict(config))
    return _CFG


def config() -> DatasetConfig:
    if _CFG is None:
        raise RuntimeError(
            "dataset_adapter is not configured.\n"
            "  Call dataset_adapter.configure('your_dataset.json') before using it.\n"
            "  See docs/dataset_config.example.json for the schema.\n"
            "  The dissertation's own config is not published -- it describes\n"
            "  human-subject recordings that are not part of this release."
        )
    return _CFG


def sessions() -> list[str]:
    return list(config().sessions)


def unavailable_sessions() -> dict[str, str]:
    """Sessions excluded from analysis, mapped to why.

    Recorded rather than silently dropped: a run manifest that lists what was
    excluded and on what grounds can be audited; one that only lists what was
    kept cannot be distinguished from a run where those sessions never existed.
    """
    return dict(config().unavailable_sessions)


# --- paths ----------------------------------------------------------------

def imu_path(s: str) -> Path:
    return config().path("imu", s)


def mocap_path(s: str) -> Path:
    return config().path("mocap", s)


def manifest_path(s: str) -> Path:
    return config().path("manifest", s)


def trial_id(s: str) -> str:
    return config().trial_id(s)


# --- loading and fusion ---------------------------------------------------

def fusion_args(session: str, imu: Path, filter_name: str):
    placement = pm.resolve_placement(trial_id=trial_id(session), raw_path=imu)
    return fiv.make_args(filter=filter_name, **placement.fusion_kwargs())


def load_res(session: str, imu: Path) -> fiv.FiveImuResult:
    """Run the five-IMU pipeline with the tolerant (bad-quaternion-interpolating) loader."""
    original = fiv.load_five_streams
    fiv.load_five_streams = sr.tolerant_load_five_streams
    try:
        return fiv.run_pipeline(imu, fusion_args(session, imu, "sflp"))
    finally:
        fiv.load_five_streams = original


def _sflp_usable(res: fiv.FiveImuResult, check: str) -> bool:
    """Is the on-chip SFLP stream good enough to use for this session?

    Two checks, because the failure modes differ: a sensor can emit
    structurally broken quaternions (caught by ``bend_quat_finite``) or emit
    well-formed but frozen ones (caught by ``twist_variance``). A frozen stream
    passes every finiteness test and still carries no information.
    """
    if check == "bend_quat_finite":
        q = res.relations["sacrum_to_upper"].q_rel
        if q is None or not len(q):
            return False
        finite = np.isfinite(q).all(axis=1)
        return bool(finite.mean() >= 0.90 and np.median(np.linalg.norm(q[finite], axis=1)) > 0.5)
    if check == "twist_variance":
        tw = res.relations["sacrum_to_sternum"].twist_deg
        return bool(np.all(np.isfinite(tw)) and float(np.nanstd(tw)) > 1e-3)
    raise ValueError(f"unknown sflp_fallback check: {check!r}")


def load_res_for_subject(session: str, imu: Path) -> fiv.FiveImuResult:
    """SFLP first, with a VQF fallback for sessions whose SFLP stream is unreliable."""
    res = load_res(session, imu)
    check = config().sflp_fallback.get(session)
    if check is None or _sflp_usable(res, check):
        return res
    print(f"session {session}: SFLP unusable ({check}) -> VQF fallback")
    return fiv.run_pipeline(imu, fusion_args(session, imu, "vqf"))


# --- manifest parsing -----------------------------------------------------

def _canon_label(label: str) -> str:
    return str(label).strip().replace(" ", "_")


def _manifest_blocks(man: dict):
    blocks = man["blocks"]
    if isinstance(blocks, list):
        return [(b["id"], b) for b in blocks]
    return list(blocks.items())


def _raw_path(raw: str | Path) -> Path:
    """Resolve a raw-segment reference recorded in a manifest.

    Manifests may carry absolute paths from the machine that produced them, so
    fall back to resolving by basename under the configured root. Ambiguity is
    an error rather than a guess -- picking the wrong segment silently changes
    which recording the analysis ran on.
    """
    root = config().root
    p = Path(raw)
    if p.is_absolute() or p.exists():
        return p
    candidate = root / p
    if candidate.exists():
        return candidate
    basename = Path(str(raw).replace("\\", "/")).name
    matches = sorted(root.glob(f"**/{basename}"))
    if len(matches) == 1:
        return matches[0]
    if len(matches) > 1:
        formal = [path for path in matches if "_staging" not in path.parts]
        if len(formal) == 1:
            return formal[0]
        raise ValueError(f"ambiguous raw segment basename {basename!r}: {matches}")
    raise FileNotFoundError(f"cannot resolve raw segment {raw!r} (basename {basename!r}) under {root}")


def _segments(man: dict):
    segs = man.get("segments")
    if isinstance(segs, dict):
        return {sid: {"raw": _raw_path(seg["raw"]), "clock": seg["clock"]} for sid, seg in segs.items()}
    if isinstance(segs, list):
        return {
            seg["id"]: {"raw": _raw_path(seg["source"]), "clock": {"a": seg["a"], "b": seg["b"]},
                        "mocap_coverage_s": seg["mocap_coverage_s"]}
            for seg in segs
        }
    return None


def _pick_segment(segs: dict, window: tuple[float, float]) -> str:
    mid = 0.5 * (window[0] + window[1])
    for sid, seg in segs.items():
        cov = seg.get("mocap_coverage_s")
        if cov and float(cov[0]) <= mid <= float(cov[1]):
            return sid
    raise ValueError(f"no segment covers mocap window {window}")


def _block_window(block: dict, override: dict) -> tuple[float, float]:
    """Block time window, preferring the most specific field the manifest offers."""
    if "window" in override:
        return tuple(float(x) for x in override["window"])
    for key in ("covered_window_s", "protocol_window_s", "window_s"):
        if key in block:
            return tuple(float(x) for x in block[key])
    return float(block["mocap_start_s"]), float(block["mocap_end_s"])


def _resolve_block(session: str, bid: str, block: dict, override: dict):
    label = _canon_label(override.get("label", block["label"]))
    if label not in LABELS:
        return None
    window = _block_window(block, override)
    sigkey = override.get("sig", block.get("mocap_signal", block.get("primary_signal", SIG_BY_LABEL[label])))
    sign = int(override.get(
        "sign",
        block.get("mocap_primary_sign", config().sign_overrides.get((session, bid), SIGN_BY_LABEL[label])),
    ))
    return label, window, sigkey, sign


def subject_blocks(
    session: str,
    tm: np.ndarray,
    msig: dict[str, np.ndarray],
) -> Iterator[tuple[str, dict, fiv.FiveImuResult, float, float, list[tuple[float, float]]]]:
    """Yield ``(block_id, block, res, a, b, bouts)`` for each labelled block.

    Two manifest shapes are supported. A single-log session has one fused result
    and one top-level clock. A segmented session -- a recording split by a
    timestamp reset, or captured in several parts -- names a segment per block,
    and each segment carries its own raw log and its own clock fit. Mixing those
    up produces bouts indexed against the wrong clock, which looks like poor
    synchronisation rather than like the bug it is.

    Bouts come from ``session_recipe.peak_bouts`` in both cases, so segmented
    and single-log sessions stay comparable.
    """
    cfg = config()
    man = json.loads(manifest_path(session).read_text(encoding="utf-8"))
    blocks = _manifest_blocks(man)
    overrides = cfg.block_overrides.get(session, {})
    segs = _segments(man)

    if segs:
        cache: dict[str, fiv.FiveImuResult] = {}
        for bid, block in blocks:
            resolved = _resolve_block(session, bid, block, overrides.get(bid, {}))
            if resolved is None:
                continue
            _, window, sigkey, sign = resolved
            seg = block.get("segment") or _pick_segment(segs, window)
            if seg not in cache:
                cache[seg] = load_res_for_subject(session, segs[seg]["raw"])
            clk = segs[seg]["clock"]
            bouts = sr.peak_bouts(tm, msig[sigkey], list(window), sign, 6.0, frac=0.35)
            yield bid, block, cache[seg], float(clk["a"]), float(clk["b"]), bouts
    else:
        res = load_res_for_subject(session, imu_path(session))
        clk = man["clock"]
        a, b = float(clk["a"]), float(clk["b"])
        for bid, block in blocks:
            resolved = _resolve_block(session, bid, block, overrides.get(bid, {}))
            if resolved is None:
                continue
            _, window, sigkey, sign = resolved
            bouts = sr.peak_bouts(tm, msig[sigkey], list(window), sign, 6.0, frac=0.35)
            yield bid, block, res, a, b, bouts


# --- quality --------------------------------------------------------------

def block_quality(block: dict) -> str:
    """Quality tier from the manifest's verdict and flags."""
    verdict = str(block.get("verdict", ""))
    flags = " ".join(block.get("flags", []))
    if "not_clean" in verdict or "not_clean" in flags:
        return "not_clean"
    if "low_n" in verdict or "low_n" in flags or "low_confidence" in verdict:
        return "low_conf"
    if "limitation" in verdict:
        return "limitation"
    return "clean"


def quality_for(session: str, bid: str, block: dict) -> str:
    """Quality tier, letting the config override what the manifest says."""
    return config().block_overrides.get(session, {}).get(bid, {}).get("quality", block_quality(block))


# --- small numeric helpers used across the analysis scripts ---------------

def peak_signed(x: np.ndarray) -> float:
    """Largest-magnitude value, keeping its sign."""
    if len(x) == 0:
        return 0.0
    return float(x[int(np.argmax(np.abs(x)))])


def n_sign_changes(x: np.ndarray) -> int:
    """Count direction reversals, ignoring numerically flat steps."""
    if len(x) < 3:
        return 0
    d = np.diff(x)
    d = d[np.abs(d) > 1e-6]
    if len(d) < 2:
        return 0
    return int(np.count_nonzero(np.diff(np.sign(d)) != 0))


# Back-compat aliases for the private names the analysis scripts used.
_peak_signed = peak_signed
_n_sign_changes = n_sign_changes


def __getattr__(name: str):
    """Expose the config's cohort data as module attributes.

    Analysis scripts read ``SUBJECTS`` and ``BLOCK_OVERRIDES`` as if they were
    constants. They are not -- they belong to whichever dataset is configured --
    so they resolve through the active config instead of being baked in here.
    """
    if name == "SUBJECTS":
        return config().sessions
    if name == "BLOCK_OVERRIDES":
        return config().block_overrides
    if name == "SIGN_OVERRIDES":
        return config().sign_overrides
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
