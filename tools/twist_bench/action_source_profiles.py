from __future__ import annotations

"""Build per-subject seven-source action-excursion profiles.

The x-axis is the six prescribed actions.  Each point is the median across the
canonical coverage-scored bouts of the maximum neutral-referenced excursion in
that bout; whiskers are Q1--Q3.  Bending actions use swing magnitude.  Twist
actions use short-window, per-bout-tared axial excursion and are explicitly not
absolute yaw.

Sources are MoCap thorax, MoCap pelvis and all five physical IMUs.  The MoCap
frames are surface-marker segment proxies, not vertebral or Cobb angles.
"""

import argparse
import csv
import hashlib
import json
import platform
import sys
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import dataset_adapter as bd  # noqa: E402
import placement_maps as pm  # noqa: E402
import session_recipe as t06  # noqa: E402
import sync_audit as sa  # noqa: E402
import twist_bench_fusion as pf  # noqa: E402
import validation3_cluster_orientation as v3  # noqa: E402


# Cohort comes from the dataset config, not from a constant here -- a hard-coded
# session list makes the script describe one study rather than one method.
ACTIONS = ["flexion", "extension", "left_bend", "right_bend", "left_twist", "right_twist"]
ACTION_LABELS = {
    "flexion": "Flexion",
    "extension": "Extension",
    "left_bend": "Left bend",
    "right_bend": "Right bend",
    "left_twist": "Left twist",
    "right_twist": "Right twist",
}
ROLE_BY_IMU = {
    "IMU0": "sternum",
    "IMU1": "sacrum",
    "IMU2": "lower",
    "IMU3": "mid",
    "IMU4": "upper",
}
SOURCES = [
    "mocap_thorax",
    "mocap_pelvis",
    "IMU0_sternum",
    "IMU1_sacrum",
    "IMU2_lower",
    "IMU3_mid",
    "IMU4_upper",
]
SOURCE_LABELS = {
    "mocap_thorax": "MoCap thorax",
    "mocap_pelvis": "MoCap pelvis",
    "IMU0_sternum": "IMU0 sternum",
    "IMU1_sacrum": "IMU1 sacrum/S1",
    "IMU2_lower": "IMU2 lower",
    "IMU3_mid": "IMU3 mid",
    "IMU4_upper": "IMU4 upper",
}
NEUTRAL_PRE_S = (-1.2, -0.2)
TWIST_AXIS = np.array([0.0, 0.0, 1.0])


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    return v / np.where(n > 1e-12, n, np.nan)


def _marker(mk: dict[str, np.ndarray], name: str) -> np.ndarray:
    key = "Trunk:" + name
    if key not in mk:
        raise KeyError(key)
    return mk[key]


def _midpoint(mk: dict[str, np.ndarray], a: str, b: str) -> np.ndarray:
    return (_marker(mk, a) + _marker(mk, b)) / 2.0


def _first_marker_name(mk: dict[str, np.ndarray], names: tuple[str, ...]) -> str:
    for name in names:
        if "Trunk:" + name in mk:
            return name
    raise KeyError(f"none of {names!r}")


def _valid_marker_mask(mk: dict[str, np.ndarray], names: list[str]) -> np.ndarray:
    out = np.ones(len(next(iter(mk.values()))), dtype=bool)
    for name in names:
        out &= np.isfinite(_marker(mk, name)).all(axis=1)
    return out


def _frames_from_markers(
    mk: dict[str, np.ndarray],
    mk_raw: dict[str, np.ndarray],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, dict[str, np.ndarray]]:
    """Return thorax/pelvis local-to-world frames and raw marker-valid masks."""
    fwd_p, up_p, ml_p = sd.pelvis_basis(mk)
    pelvis = np.stack([ml_p, fwd_p, up_p], axis=-1)

    mid_spine_name = _first_marker_name(mk, ("T7", "T8"))
    upper = _midpoint(mk, "C7 (2)", "T2")
    lower = _midpoint(mk, "T11", "L1")
    up_t = _unit(upper - lower)
    posterior = (
        _marker(mk, "C7 (2)")
        + _marker(mk, "T2")
        + _marker(mk, mid_spine_name)
        + _marker(mk, "T11")
    ) / 4.0
    sternum = _midpoint(mk, "JN", "XP")
    fwd_raw = sternum - posterior
    fwd_t = _unit(fwd_raw - np.sum(fwd_raw * up_t, axis=1, keepdims=True) * up_t)
    ml_t = _unit(np.cross(fwd_t, up_t))
    fwd_t = _unit(np.cross(up_t, ml_t))
    thorax = np.stack([ml_t, fwd_t, up_t], axis=-1)

    pelvis_names = ["LPSIS_F", "RPSIS_F", "LPSIS_B", "RPSIS_B"]
    thorax_names = ["JN", "XP", "C7 (2)", "T2", mid_spine_name, "T11", "L1"]
    valid = {
        "mocap_thorax": _valid_marker_mask(mk_raw, thorax_names),
        "mocap_pelvis": _valid_marker_mask(mk_raw, pelvis_names),
    }
    det_t = np.linalg.det(thorax)
    det_p = np.linalg.det(pelvis)
    if np.nanmedian(det_t) < 0.99 or np.nanmedian(det_p) < 0.99:
        raise ValueError("MoCap frame handedness/orthonormality check failed")
    return thorax, pelvis, fwd_p, up_p, {**valid, "ml_p": ml_p}


def load_mocap(subject: str) -> dict[str, Any]:
    path = bd.mocap_path(subject)
    tm, mk = sa.parse_motive_markers(path, gap_fill=True)
    tm_raw, mk_raw = sa.parse_motive_markers(path, gap_fill=False)
    if len(tm) != len(tm_raw) or not np.allclose(tm, tm_raw):
        raise ValueError(f"MoCap parse mismatch for {path}")
    thorax, pelvis, fwd_p, up_p, aux = _frames_from_markers(mk, mk_raw)
    ml_p = aux.pop("ml_p")

    upper = _midpoint(mk, "C7 (2)", "T2")
    sternum = _midpoint(mk, "JN", "XP")
    sacral_name = _first_marker_name(mk, ("S2", "S1"))
    sacrum = (
        _marker(mk, "L3")
        + _marker(mk, sacral_name)
        + _marker(mk, "LPSIS_B")
        + _marker(mk, "RPSIS_B")
    ) / 4.0
    still = tm <= 8.0
    flex, lat = v3.chord_tilt(sacrum, upper, fwd_p, up_p, ml_p, still)
    axial = v3.axial(sternum, sacrum, fwd_p, ml_p, still)
    lat_dominant = (np.abs(lat) > 15.0) & (np.abs(flex) < 10.0)
    if int(np.sum(lat_dominant)) > 50:
        axial = axial - np.polyfit(lat[lat_dominant], axial[lat_dominant], 1)[0] * lat
    return {
        "path": path,
        "t": tm,
        "frames": {"mocap_thorax": thorax, "mocap_pelvis": pelvis},
        "valid": aux,
        "signals": {"flex": flex, "lat": lat, "axial": axial},
    }


def _metric_from_wxyz(q: np.ndarray, action: str) -> float:
    twist, swing = pf.swing_twist_deg(pf.qnormalize(q), TWIST_AXIS)
    if "twist" in action:
        twist = pf.unwrap_deg(twist)
        return float(np.nanmax(np.abs(twist)))
    return float(np.nanmax(swing))


def frame_excursion(
    t: np.ndarray,
    frames: np.ndarray,
    raw_valid: np.ndarray,
    lo: float,
    hi: float,
    action: str,
) -> tuple[float | None, dict[str, float]]:
    neutral = (t >= lo + NEUTRAL_PRE_S[0]) & (t <= lo + NEUTRAL_PRE_S[1])
    movement = (t >= lo) & (t <= hi)
    support = {
        "neutral_support_fraction": float(np.mean(raw_valid[neutral])) if np.any(neutral) else 0.0,
        "movement_support_fraction": float(np.mean(raw_valid[movement])) if np.any(movement) else 0.0,
    }
    nsel = neutral & raw_valid
    msel = movement & raw_valid
    if int(np.count_nonzero(nsel)) < 3:
        nsel = neutral
    if int(np.count_nonzero(msel)) < 3:
        msel = movement
    if int(np.count_nonzero(nsel)) < 3 or int(np.count_nonzero(msel)) < 3:
        return None, support
    r0 = Rotation.from_matrix(frames[nsel]).mean()
    rel = r0.inv() * Rotation.from_matrix(frames[msel])
    xyzw = rel.as_quat()
    wxyz = np.column_stack([xyzw[:, 3], xyzw[:, 0], xyzw[:, 1], xyzw[:, 2]])
    return _metric_from_wxyz(wxyz, action), support


def imu_excursion(
    res,
    q_segment: np.ndarray,
    a: float,
    b: float,
    lo: float,
    hi: float,
    action: str,
) -> tuple[float | None, dict[str, float]]:
    t = res.t_s
    neutral = (t >= a * (lo + NEUTRAL_PRE_S[0]) + b) & (t <= a * (lo + NEUTRAL_PRE_S[1]) + b)
    movement = (t >= a * lo + b) & (t <= a * hi + b)
    support = {
        "neutral_support_fraction": 1.0 if int(np.count_nonzero(neutral)) >= 3 else 0.0,
        "movement_support_fraction": 1.0 if int(np.count_nonzero(movement)) >= 3 else 0.0,
    }
    if int(np.count_nonzero(neutral)) < 3 or int(np.count_nonzero(movement)) < 3:
        return None, support
    q0 = pf.quat_average(q_segment[neutral])
    qrel = pf.qmul(pf.qconj(q0)[None, :], q_segment[movement])
    return _metric_from_wxyz(qrel, action), support


def _quartiles(values: list[float]) -> tuple[float | None, float | None, float | None]:
    if not values:
        return None, None, None
    q = np.percentile(np.asarray(values, dtype=float), [25.0, 50.0, 75.0])
    return tuple(round(float(v), 6) for v in (q[1], q[0], q[2]))


def build_rows(subjects: list[str], validation: dict[str, Any]) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    provenance: dict[str, Any] = {}
    for subject in subjects:
        trial = f"T{subject}_P{subject}"
        moc = load_mocap(subject)
        subject_payload = validation["subjects"][trial]
        block_authority = {b["block"]: b for b in subject_payload["blocks"]}
        placement = pm.resolve_placement(trial_id=trial)
        seen_blocks: set[str] = set()
        input_paths: set[Path] = {moc["path"], bd.manifest_path(subject)}

        for bid, _, res, a, b, _ in bd.subject_blocks(subject, moc["t"], moc["signals"]):
            if bid not in block_authority:
                raise KeyError(f"{trial}/{bid} absent from corrected validation authority")
            seen_blocks.add(bid)
            auth = block_authority[bid]
            action = str(auth["label"])
            if action not in ACTIONS:
                continue
            input_paths.add(Path(res.summary["input"]))
            sensor_by_imu = {state.imu.upper(): state for state in res.sensors.values()}
            if set(sensor_by_imu) != set(ROLE_BY_IMU):
                raise ValueError(f"{trial}/{bid}: missing physical IMU streams {set(sensor_by_imu)}")

            for bout_index, (lo, hi) in enumerate(auth["scored_bouts"]):
                for source in ("mocap_thorax", "mocap_pelvis"):
                    value, support = frame_excursion(
                        moc["t"], moc["frames"][source], moc["valid"][source],
                        float(lo), float(hi), action,
                    )
                    rows.append({
                        "subject": f"T{subject}", "trial_id": trial, "block": bid,
                        "action": action, "bout_index": bout_index, "source": source,
                        "source_label": SOURCE_LABELS[source],
                        "metric_type": "axial_twist_excursion" if "twist" in action else "swing_excursion",
                        "excursion_deg": value, "mocap_start_s": float(lo), "mocap_end_s": float(hi),
                        "quality": auth["quality"], **support,
                    })

                for imu, role in ROLE_BY_IMU.items():
                    source = f"{imu}_{role}"
                    value, support = imu_excursion(
                        res, sensor_by_imu[imu].q_segment, float(a), float(b),
                        float(lo), float(hi), action,
                    )
                    rows.append({
                        "subject": f"T{subject}", "trial_id": trial, "block": bid,
                        "action": action, "bout_index": bout_index, "source": source,
                        "source_label": SOURCE_LABELS[source],
                        "metric_type": "axial_twist_excursion" if "twist" in action else "swing_excursion",
                        "excursion_deg": value, "mocap_start_s": float(lo), "mocap_end_s": float(hi),
                        "quality": auth["quality"], **support,
                    })

        if seen_blocks != set(block_authority):
            raise ValueError(f"{trial}: block mismatch: seen={sorted(seen_blocks)} authority={sorted(block_authority)}")
        provenance[trial] = {
            "mapping_status": placement.status,
            "mapping_sha256": placement.canonical_sha256,
            "role_to_imu": dict(placement.role_to_imu),
            "input_files": [
                {"path": str(p.resolve()), "sha256": sha256_file(p.resolve()), "size_bytes": p.resolve().stat().st_size}
                for p in sorted(input_paths, key=str)
            ],
        }
    return rows, provenance


def aggregate_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    grouped: dict[tuple[str, str, str], list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        grouped[(row["subject"], row["action"], row["source"])].append(row)
    out: list[dict[str, Any]] = []
    subjects = sorted({row["subject"] for row in rows})
    for subject in subjects:
        for action in ACTIONS:
            for source in SOURCES:
                group = grouped.get((subject, action, source), [])
                usable = [
                    r for r in group
                    if r["excursion_deg"] is not None
                    and min(float(r["neutral_support_fraction"]), float(r["movement_support_fraction"])) >= 0.50
                ]
                values = [float(r["excursion_deg"]) for r in usable]
                median, q1, q3 = _quartiles(values)
                min_support = min(
                    [min(float(r["neutral_support_fraction"]), float(r["movement_support_fraction"])) for r in group]
                    or [0.0]
                )
                out.append({
                    "subject": subject,
                    "action": action,
                    "action_label": ACTION_LABELS[action],
                    "source": source,
                    "source_label": SOURCE_LABELS[source],
                    "median_excursion_deg": median,
                    "q1_excursion_deg": q1,
                    "q3_excursion_deg": q3,
                    "n_bouts": len(values),
                    "n_bouts_total": len(group),
                    "min_raw_support_fraction": round(min_support, 6),
                    "support_flag": (
                        "no_canonical_bouts" if not group
                        else (
                            "insufficient_support" if not values
                            else ("ok" if min_support >= 0.95 else "low_support")
                        )
                    ),
                })
    return out


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        raise ValueError("no rows")
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot_subject(path: Path, subject: str, aggregate: list[dict[str, Any]]) -> None:
    rows = [r for r in aggregate if r["subject"] == subject]
    lookup = {(r["source"], r["action"]): r for r in rows}
    x = np.arange(len(ACTIONS), dtype=float)
    colors = {
        "mocap_thorax": "#111111", "mocap_pelvis": "#777777",
        "IMU0_sternum": "#d62728", "IMU1_sacrum": "#1f77b4",
        "IMU2_lower": "#2ca02c", "IMU3_mid": "#9467bd", "IMU4_upper": "#ff7f0e",
    }
    markers = {"mocap_thorax": "o", "mocap_pelvis": "s"}
    fig, ax = plt.subplots(figsize=(13.2, 7.2), constrained_layout=True)
    for source in SOURCES:
        vals = [lookup[(source, action)]["median_excursion_deg"] for action in ACTIONS]
        q1 = [lookup[(source, action)]["q1_excursion_deg"] for action in ACTIONS]
        q3 = [lookup[(source, action)]["q3_excursion_deg"] for action in ACTIONS]
        y = np.array([np.nan if v is None else float(v) for v in vals])
        low = np.array([0.0 if v is None or a is None else float(v) - float(a) for v, a in zip(vals, q1)])
        high = np.array([0.0 if v is None or a is None else float(a) - float(v) for v, a in zip(vals, q3)])
        ax.errorbar(
            x, y, yerr=np.vstack([low, high]), color=colors[source],
            marker=markers.get(source, "D"), markersize=5.0 if source.startswith("mocap") else 4.2,
            linewidth=2.2 if source.startswith("mocap") else 1.35,
            linestyle="-" if source.startswith("mocap") else "--",
            capsize=2.5, alpha=0.95, label=SOURCE_LABELS[source],
        )
    low_support = sum(r["support_flag"] == "low_support" and r["source"].startswith("mocap") for r in rows)
    no_coverage_actions = sorted({
        r["action_label"] for r in rows if r["support_flag"] == "no_canonical_bouts"
    })
    ax.set_xticks(x, [ACTION_LABELS[a] for a in ACTIONS])
    ax.set_ylabel("Neutral-referenced excursion (deg)")
    ax.set_title(
        f"{subject}: seven-source six-action profile\n"
        "Bend = swing magnitude; twist = short-window tared axial excursion (not absolute yaw)"
    )
    ax.grid(axis="y", alpha=0.22)
    ax.set_ylim(bottom=0.0)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.12), ncol=4, frameon=False)
    notes = []
    if low_support:
        notes.append(f"{low_support} MoCap aggregate points include <95% raw marker support")
    if no_coverage_actions:
        notes.append("no canonical bouts: " + ", ".join(no_coverage_actions))
    if notes:
        ax.text(
            0.995, 0.01, "; ".join(notes),
            transform=ax.transAxes, ha="right", va="bottom", fontsize=8, color="#8c4a00",
        )
    fig.savefig(path.with_suffix(".png"), dpi=170)
    fig.savefig(path.with_suffix(".svg"))
    plt.close(fig)


def plot_cohort(path: Path, aggregate: list[dict[str, Any]]) -> None:
    subjects = sorted({r["subject"] for r in aggregate})
    fig, axes = plt.subplots(5, 3, figsize=(18, 24), sharex=True, constrained_layout=True)
    axes = axes.ravel()
    colors = ["#111111", "#777777", "#d62728", "#1f77b4", "#2ca02c", "#9467bd", "#ff7f0e"]
    for ax, subject in zip(axes, subjects):
        lookup = {(r["source"], r["action"]): r for r in aggregate if r["subject"] == subject}
        for source, color in zip(SOURCES, colors):
            y = [lookup[(source, action)]["median_excursion_deg"] for action in ACTIONS]
            ax.plot(
                np.arange(6), [np.nan if v is None else v for v in y],
                color=color, lw=1.5 if source.startswith("mocap") else 0.9,
                marker="o", ms=2.5, alpha=0.9,
            )
        ax.set_title(subject)
        ax.grid(axis="y", alpha=0.18)
        ax.set_ylim(bottom=0.0)
    for ax in axes[len(subjects):]:
        ax.axis("off")
    for ax in axes[-3:]:
        ax.set_xticks(np.arange(6), ["Flex", "Ext", "LB", "RB", "LT", "RT"])
    for ax in axes[::3]:
        ax.set_ylabel("deg")
    handles = [plt.Line2D([], [], color=c, marker="o", lw=1.5, label=SOURCE_LABELS[s]) for s, c in zip(SOURCES, colors)]
    fig.legend(handles=handles, loc="lower center", ncol=4, frameon=False)
    fig.suptitle("SpineSense seven-source neutral-referenced action profiles", fontsize=16)
    fig.savefig(path.with_suffix(".png"), dpi=150)
    fig.savefig(path.with_suffix(".svg"))
    plt.close(fig)


def write_interactive(path: Path, aggregate: list[dict[str, Any]]) -> None:
    compact = [
        {
            "subject": r["subject"], "action": r["action"], "source": r["source"],
            "median": r["median_excursion_deg"], "q1": r["q1_excursion_deg"],
            "q3": r["q3_excursion_deg"], "n": r["n_bouts"], "support": r["min_raw_support_fraction"],
        }
        for r in aggregate
    ]
    data_json = json.dumps(compact, separators=(",", ":"), ensure_ascii=False)
    subject_options = "".join(f'<option value="{s}">{s}</option>' for s in sorted({r["subject"] for r in aggregate}))
    fragment = f'''<div id="spinesense-action-source-profiles">
  <div class="viz-controls">
    <label class="form-label" for="profile-subject">Subject
      <select class="form-select" id="profile-subject">{subject_options}</select>
    </label>
  </div>
  <svg class="profile-chart" viewBox="0 0 760 430" role="img" aria-labelledby="profile-title profile-desc">
    <title id="profile-title">Seven-source six-action profile</title>
    <desc id="profile-desc">Median neutral-referenced angular excursion with interquartile whiskers for MoCap thorax, MoCap pelvis and five IMUs.</desc>
  </svg>
  <div class="profile-legend text-small" aria-label="Series legend"></div>
  <div class="profile-detail text-small text-muted" aria-live="polite"></div>
</div>
<style>
#spinesense-action-source-profiles {{ width:100%; color:var(--foreground); }}
#spinesense-action-source-profiles .profile-chart {{ width:100%; height:auto; display:block; }}
#spinesense-action-source-profiles .axis {{ stroke:var(--border); stroke-width:1; }}
#spinesense-action-source-profiles .grid {{ stroke:var(--border); stroke-width:1; opacity:.55; }}
#spinesense-action-source-profiles .tick-text {{ fill:var(--muted-foreground); font-size:11px; }}
#spinesense-action-source-profiles .axis-label {{ fill:var(--foreground); font-size:12px; }}
#spinesense-action-source-profiles .profile-legend {{ display:flex; flex-wrap:wrap; gap:8px 16px; justify-content:center; margin-top:4px; }}
#spinesense-action-source-profiles .legend-item {{ display:inline-flex; align-items:center; gap:6px; }}
#spinesense-action-source-profiles .legend-line {{ width:24px; height:0; border-top:2px solid currentColor; }}
#spinesense-action-source-profiles .profile-detail {{ min-height:20px; text-align:center; margin-top:6px; }}
#spinesense-action-source-profiles .mark {{ cursor:pointer; }}
@media (max-width:520px) {{ #spinesense-action-source-profiles .tick-text {{ font-size:10px; }} }}
</style>
<script>
(() => {{
  const root=document.getElementById('spinesense-action-source-profiles');
  const svg=root.querySelector('.profile-chart');
  const select=root.querySelector('#profile-subject');
  const legend=root.querySelector('.profile-legend');
  const detail=root.querySelector('.profile-detail');
  const data={data_json};
  const actions=['flexion','extension','left_bend','right_bend','left_twist','right_twist'];
  const actionLabels={{flexion:'Flexion',extension:'Extension',left_bend:'Left bend',right_bend:'Right bend',left_twist:'Left twist',right_twist:'Right twist'}};
  const sources={json.dumps(SOURCES)};
  const labels={json.dumps(SOURCE_LABELS)};
  const styles=[
    ['var(--foreground)',''],['var(--muted-foreground)','4 3'],['var(--viz-series-1)',''],
    ['var(--viz-series-2)','5 3'],['var(--viz-series-3)',''],['var(--viz-series-4)','5 3'],['var(--viz-series-5)','']
  ];
  const NS='http://www.w3.org/2000/svg';
  const el=(name,attrs={{}},text='')=>{{const n=document.createElementNS(NS,name);Object.entries(attrs).forEach(([k,v])=>n.setAttribute(k,v));if(text)n.textContent=text;return n;}};
  const draw=()=>{{
    svg.replaceChildren();
    const rows=data.filter(d=>d.subject===select.value);
    const valid=rows.map(d=>d.q3).filter(v=>v!==null);
    const ymax=Math.max(10,Math.ceil(Math.max(...valid,10)/10)*10);
    const left=58,right=738,top=18,bottom=355;
    const x=i=>left+35+i*(right-left-70)/5;
    const y=v=>bottom-(v/ymax)*(bottom-top);
    for(let k=0;k<=5;k++){{const v=ymax*k/5,yy=y(v);svg.append(el('line',{{x1:left,y1:yy,x2:right,y2:yy,class:'grid'}}));svg.append(el('text',{{x:left-9,y:yy+4,'text-anchor':'end',class:'tick-text'}},v.toFixed(0)));}}
    svg.append(el('line',{{x1:left,y1:bottom,x2:right,y2:bottom,class:'axis'}}));
    svg.append(el('line',{{x1:left,y1:top,x2:left,y2:bottom,class:'axis'}}));
    actions.forEach((a,i)=>svg.append(el('text',{{x:x(i),y:bottom+24,'text-anchor':'middle',class:'tick-text'}},actionLabels[a])));
    svg.append(el('text',{{x:(left+right)/2,y:414,'text-anchor':'middle',class:'axis-label'}},'Prescribed action'));
    const yl=el('text',{{x:15,y:(top+bottom)/2,'text-anchor':'middle',class:'axis-label',transform:`rotate(-90 15 ${{(top+bottom)/2}})`}},'Neutral-referenced excursion (deg)');svg.append(yl);
    sources.forEach((s,si)=>{{
      const sr=actions.map(a=>rows.find(d=>d.source===s&&d.action===a));
      const pts=sr.map((d,i)=>d&&d.median!==null?`${{x(i)}},${{y(d.median)}}`:null);
      let seg=[]; const flush=()=>{{if(seg.length>1)svg.append(el('polyline',{{points:seg.join(' '),fill:'none',stroke:styles[si][0],'stroke-width':s.startsWith('mocap')?2.6:1.8,'stroke-dasharray':styles[si][1]}}));seg=[];}};
      pts.forEach(p=>{{if(p)seg.push(p);else flush();}});flush();
      sr.forEach((d,i)=>{{if(!d||d.median===null)return;const xx=x(i),yy=y(d.median);if(d.q1!==null&&d.q3!==null){{svg.append(el('line',{{x1:xx,y1:y(d.q1),x2:xx,y2:y(d.q3),stroke:styles[si][0],'stroke-width':1}}));svg.append(el('line',{{x1:xx-3,y1:y(d.q1),x2:xx+3,y2:y(d.q1),stroke:styles[si][0]}}));svg.append(el('line',{{x1:xx-3,y1:y(d.q3),x2:xx+3,y2:y(d.q3),stroke:styles[si][0]}}));}}
        const low=d.support<0.95;
        const mark=el(si<2?'circle':'rect',si<2?{{cx:xx,cy:yy,r:4,fill:low?'var(--background)':styles[si][0],stroke:styles[si][0],'stroke-width':low?2:0,class:'mark'}}:{{x:xx-3.5,y:yy-3.5,width:7,height:7,fill:low?'var(--background)':styles[si][0],stroke:styles[si][0],'stroke-width':low?2:0,class:'mark'}});
        mark.addEventListener('click',()=>{{detail.textContent=`${{select.value}} · ${{actionLabels[d.action]}} · ${{labels[d.source]}}: ${{d.median.toFixed(1)}}° (IQR ${{d.q1.toFixed(1)}}–${{d.q3.toFixed(1)}}°, n=${{d.n}}, minimum raw support ${{(100*d.support).toFixed(0)}}%)`;}});svg.append(mark);
      }});
    }});
    detail.textContent=`${{select.value}} · bending points are swing magnitude; twist points are short-window tared axial excursion, not absolute yaw.`;
  }};
  legend.innerHTML=sources.map((s,i)=>`<span class="legend-item"><span class="legend-line" style="color:${{styles[i][0]}};border-top-style:${{styles[i][1]?'dashed':'solid'}}"></span>${{labels[s]}}</span>`).join('');
  select.addEventListener('change',draw);draw();
}})();
</script>'''
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(fragment, encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--subjects", nargs="*", default=bd.sessions())
    parser.add_argument(
        "--validation",
        type=Path,
        default=HERE / "runs/mapping_repair_2026-07-13/C_corrected_uniform/validation/cohort_validation.json",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=HERE / "runs/action_source_profiles_2026-07-13",
    )
    parser.add_argument("--interactive-html", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    subjects = [f"{int(s):02d}" for s in args.subjects]
    bad = sorted(set(subjects) - set(bd.sessions()))
    if bad:
        raise SystemExit(f"No canonical corrected-uniform block authority for subjects: {bad}")
    out_dir = args.out_dir.resolve()
    if out_dir.exists() and any(out_dir.iterdir()):
        raise SystemExit(f"refusing to overwrite non-empty output directory: {out_dir}")
    out_dir.mkdir(parents=True, exist_ok=True)
    plots = out_dir / "plots"
    plots.mkdir()
    validation_path = args.validation.resolve()
    validation = json.loads(validation_path.read_text(encoding="utf-8"))
    if validation.get("run_type") != "corrected_uniform_validation":
        raise SystemExit(f"wrong validation run_type: {validation.get('run_type')}")

    rows, trial_provenance = build_rows(subjects, validation)
    aggregate = aggregate_rows(rows)
    _write_csv(out_dir / "bout_source_excursions.csv", rows)
    _write_csv(out_dir / "subject_action_source_summary.csv", aggregate)
    for subject in sorted({r["subject"] for r in aggregate}):
        plot_subject(plots / f"{subject}_seven_source_profile", subject, aggregate)
    plot_cohort(plots / "cohort_seven_source_profiles", aggregate)
    aggregate_json = out_dir / "subject_action_source_summary.json"
    aggregate_json.write_text(json.dumps(aggregate, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    unavailable = bd.unavailable_sessions()
    (out_dir / "unavailable_trials.json").write_text(
        json.dumps(unavailable, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    if args.interactive_html:
        write_interactive(args.interactive_html.resolve(), aggregate)

    outputs = {}
    for path in sorted(out_dir.rglob("*")):
        if path.is_file() and path.name != "manifest.json":
            outputs[str(path.relative_to(out_dir))] = {
                "sha256": sha256_file(path), "size_bytes": path.stat().st_size,
            }
    manifest = {
        "schema_version": 1,
        "run_type": "action_source_profiles_v1",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "subjects": [f"T{s}" for s in subjects],
        "unavailable_trials": unavailable,
        "mapping_registry": {"path": str(pm.DEFAULT_CONFIG_PATH.resolve()), "sha256": sha256_file(pm.DEFAULT_CONFIG_PATH.resolve())},
        "validation_authority": {"path": str(validation_path), "sha256": sha256_file(validation_path)},
        "generator": {"path": str(Path(__file__).resolve()), "sha256": sha256_file(Path(__file__).resolve())},
        "interactive_html": (
            {"path": str(args.interactive_html.resolve()), "sha256": sha256_file(args.interactive_html.resolve())}
            if args.interactive_html else None
        ),
        "definition": {
            "x_axis": ACTIONS,
            "y_axis": "neutral-referenced action-matched angular excursion in degrees",
            "point": "median of per-bout maximum excursion",
            "whisker": "bout-level Q1 to Q3",
            "neutral_window_s_relative_to_bout_start": list(NEUTRAL_PRE_S),
            "bend_metric": "swing magnitude from each source's own neutral-tared orientation",
            "twist_metric": "absolute short-window tared twist about local longitudinal z; not absolute yaw",
            "mocap_thorax": "surface-marker thorax segment frame; not thoracic curvature/vertebral/Cobb angle",
            "mocap_pelvis": "surface-marker pelvis segment frame",
            "imu_sources": ROLE_BY_IMU,
        },
        "counts": {
            "bout_source_rows": len(rows),
            "aggregate_rows": len(aggregate),
            "expected_sources_per_bout": 7,
            "expected_aggregate_rows": len(subjects) * len(ACTIONS) * len(SOURCES),
            "missing_subject_actions": sorted({
                f"{r['subject']}:{r['action']}"
                for r in aggregate if r["support_flag"] == "no_canonical_bouts"
            }),
        },
        "trials": trial_provenance,
        "environment": {"python": sys.version, "platform": platform.platform(), "numpy": np.__version__},
        "outputs": outputs,
        "claim_boundaries": [
            "twist is neutral-tared short-window excursion, not absolute yaw",
            "MoCap thorax is a surface-marker segment proxy, not static thoracic curvature",
            "connected categorical points form an action profile; inter-action slopes have no physical meaning",
            "similar profiles are QC evidence, not standalone proof of anatomical placement",
        ],
    }
    (out_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(json.dumps(manifest["counts"], indent=2))
    print(f"wrote {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
