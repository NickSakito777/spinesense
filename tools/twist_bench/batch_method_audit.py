from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import dataset_adapter as da
import five_imu_fusion as fiv
import signed_diagnostic as sd
import sync_audit as sa
import twist_bench_fusion as pf
import validation3_cluster_orientation as v3

OUT = HERE / "plots" / "sync_audit"


def paths(session: str) -> tuple[Path, Path]:
    """Resolve one session's IMU log and mocap file through the configured dataset.

    Sessions whose raw log is split or named irregularly are handled by the path
    template in the dataset config, not by a special case here.
    """
    return da.imu_path(session), da.mocap_path(session)


def separated_ratio(tops: list[dict[str, float]]) -> float:
    if len(tops) < 2 or not tops[0]["corr"]:
        return 0.0
    sep = [x for x in tops[1:] if abs(x["b"] - tops[0]["b"]) > 8.0]
    return float(sep[0]["corr"] / tops[0]["corr"]) if sep else 0.0


def sync_class(corr: float, sep: float) -> str:
    if corr < 0.75 or sep > 0.85:
        return "sync_limited"
    if corr < 0.85:
        return "medium"
    if sep > 0.70:
        return "clean_alias"
    return "clean"


def sync_rows() -> list[dict[str, object]]:
    rows = []
    for s in da.sessions():
        rec = json.loads((OUT / f"T{s}_P{s}_auto_sync.json").read_text(encoding="utf-8"))
        tops = rec["top_coarse_offsets"]
        sep = separated_ratio(tops)
        corr = float(rec["clock"]["corr"])
        rows.append({
            "subject": s,
            "corr": corr,
            "a": float(rec["clock"]["a"]),
            "b": float(rec["clock"]["b"]),
            "sep_ratio": sep,
            "class": sync_class(corr, sep),
            "top_offsets": " ".join(f"{x['b']:.1f}:{x['corr']:.2f}" for x in tops[:5]),
        })
    return rows


def covered(bouts, cov: float):
    return [b for b in bouts if b[1] <= cov - 0.3]


def movement_bouts(tm, flex, lat, axial, cov: float):
    return {
        "flex": covered(v3.segment(tm, flex, +1, (lat, axial)), cov),
        "ext": covered(v3.segment(tm, flex, -1, (lat, axial), hi=8, lo=3, opurity=12), cov),
        "latL": covered(v3.segment(tm, lat, +1, (flex, axial)), cov),
        "latR": covered(v3.segment(tm, lat, -1, (flex, axial)), cov),
        "twL": covered(v3.segment(tm, axial, +1, (flex, lat)), cov),
        "twR": covered(v3.segment(tm, axial, -1, (flex, lat)), cov),
    }


def score_fixed(tm, msig, ti, isig, bouts) -> tuple[float, float, float] | None:
    xs, ys = [], []
    for a, b in bouts:
        g = np.arange(a, b, 0.01)
        if len(g) < 5:
            continue
        m0 = i0 = 0.0
        g0 = np.arange(max(tm[0], a - 1.2), a - 0.2, 0.01)
        if len(g0) >= 5:
            m0 = float(np.mean(np.interp(g0, tm, msig)))
            i0 = float(np.mean(np.interp(g0, ti, isig)))
        xs.append(np.interp(g, ti, isig) - i0)
        ys.append(np.interp(g, tm, msig) - m0)
    if not xs:
        return None
    x = np.concatenate(xs)
    y = np.concatenate(ys)
    if np.std(x) < 1e-6 or np.std(y) < 1e-6:
        return None
    gain, offset = np.polyfit(x, y, 1)
    rmse = float(np.sqrt(np.mean((y - (gain * x + offset)) ** 2)))
    return float(np.corrcoef(x, y)[0, 1]), float(gain), rmse


def axis_samples(tm, ti, msig, x2, bouts):
    xs, ys = [], []
    for a, b in bouts:
        g = np.arange(a, b, 0.01)
        if len(g) < 5:
            continue
        x0 = np.zeros(2)
        y0 = 0.0
        g0 = np.arange(max(tm[0], a - 1.2), a - 0.2, 0.01)
        if len(g0) >= 5:
            x0 = np.array([np.mean(np.interp(g0, ti, x2[:, 0])), np.mean(np.interp(g0, ti, x2[:, 1]))])
            y0 = float(np.mean(np.interp(g0, tm, msig)))
        xs.append(np.column_stack([
            np.interp(g, ti, x2[:, 0]),
            np.interp(g, ti, x2[:, 1]),
        ]) - x0)
        ys.append(np.interp(g, tm, msig) - y0)
    if not xs:
        return None, None
    return np.vstack(xs), np.concatenate(ys)


def fit_axis(tm, ti, msig, x2, bouts):
    x, y = axis_samples(tm, ti, msig, x2, bouts)
    if x is None or len(y) < 20 or np.std(y) < 1e-6:
        return None
    w, *_ = np.linalg.lstsq(x, y, rcond=None)
    n = float(np.linalg.norm(w))
    return None if n < 1e-9 else w / n


def fmt_score(v):
    return ("", "", "") if v is None else tuple(f"{x:.3f}" for x in v)


def still_before(res, a: float, b: float, t_anchor: float, roles: tuple[str, str], look=2.5, win=0.6):
    ti = res.t_s
    anchor = a * t_anchor + b
    seg = np.where((ti >= anchor - look) & (ti < anchor - 0.15))[0]
    if len(seg) < 5:
        return ti <= 8.0
    mag = sum(np.linalg.norm(res.sensors[r].gyr_cal_dps, axis=1) for r in roles)
    n = max(3, int(win / np.median(np.diff(ti))))
    csum = np.cumsum(np.insert(mag[seg], 0, 0.0))
    best = min(range(len(seg) - n + 1), key=lambda k: csum[k + n] - csum[k])
    mask = np.zeros(len(ti), dtype=bool)
    mask[seg[best:best + n]] = True
    return mask


def qrel(qp, qc):
    return pf.qmul(pf.qconj(qp), qc)


def decompose_twist(q_rel, mask, flip=True):
    q0 = pf.quat_average(q_rel[mask])
    qt = pf.qmul(pf.qconj(q0)[None, :], q_rel)
    tw, _ = pf.swing_twist_deg(qt, pf.SEGMENT_TWIST_AXIS)
    tw = pf.unwrap_deg(tw)
    return -tw if flip else tw


def per_bout_twist(res, a: float, b: float, bouts):
    q = qrel(res.sensors["upper"].q_segment, res.sensors["sternum"].q_segment)
    series = decompose_twist(q, res.t_s <= 8.0)
    for lo, hi in bouts:
        mask = still_before(res, a, b, lo, ("upper", "sternum"))
        local = decompose_twist(q, mask)
        i0 = np.searchsorted(res.t_s, a * (lo - 1.0) + b)
        i1 = np.searchsorted(res.t_s, a * (hi + 0.6) + b)
        series[i0:i1] = local[i0:i1]
    return series


def phi_stats(res, a: float, b: float, bouts) -> tuple[float, float, float, float]:
    tw, _ = pf.swing_twist_deg(res.relations["sacrum_to_upper"].q_tared, pf.SEGMENT_TWIST_AXIS)
    tw = pf.unwrap_deg(tw)
    vals, times = [], []
    for lo, _ in bouts:
        mask = still_before(res, a, b, lo, ("sacrum", "upper"))
        vals.append(float(np.median(tw[mask])))
        times.append(lo)
    if len(vals) < 2:
        return np.nan, np.nan, np.nan, np.nan
    vals = np.array(vals)
    times = np.array(times)
    return float(vals[-1] - vals[0]), float(np.ptp(vals)), float(np.std(vals)), float(np.polyfit(times, vals, 1)[0] * 60.0)


def channel_row(row: dict[str, object]) -> dict[str, object]:
    s = str(row["subject"])
    a = float(row["a"])
    b = float(row["b"])
    imu, mocap = paths(s)
    tm, flex, lat, axial = sd.mocap_signed(mocap)
    res = fiv.run_pipeline(imu, fiv.make_args(layout_preset="t01", filter="vqf"))
    ti_m = (res.t_s - b) / a
    cov = min(float(tm[-1]), float(ti_m[-1]))
    bouts = movement_bouts(tm, flex, lat, axial, cov)
    bx, by = sd.imu_signed_bend(res.relations["sacrum_to_upper"])
    raw_bend = np.column_stack([bx, by])
    bf, bl, rmap = sd.diagnostic_bend_map(tm, flex, lat, axial, ti_m, bx, by)
    ti = ti_m
    flex_bouts = bouts["flex"] + bouts["ext"]
    lat_bouts = bouts["latL"] + bouts["latR"]
    flex_score = score_fixed(tm, flex, ti, bf, flex_bouts)
    lat_score = score_fixed(tm, lat, ti, bl, lat_bouts)
    flex_axis = fit_axis(tm, ti, flex, raw_bend, flex_bouts)
    lat_from_flex = raw_bend @ np.array([-flex_axis[1], flex_axis[0]]) if flex_axis is not None else None
    lat_held = score_fixed(tm, lat, ti, lat_from_flex, lat_bouts) if lat_from_flex is not None else None
    lat_axis = fit_axis(tm, ti, lat, raw_bend, lat_bouts)
    flex_from_lat = raw_bend @ np.array([lat_axis[1], -lat_axis[0]]) if lat_axis is not None else None
    flex_held = score_fixed(tm, flex, ti, flex_from_lat, flex_bouts) if flex_from_lat is not None else None
    tw_global = -res.relations["sacrum_to_sternum"].twist_deg
    tw_bouts = bouts["twL"] + bouts["twR"]
    tw_g_score = score_fixed(tm, axial, ti, tw_global, tw_bouts)
    bend_phi = phi_stats(res, a, b, bouts["flex"] + bouts["ext"] + bouts["latL"] + bouts["latR"])
    all_phi = phi_stats(res, a, b, bouts["flex"] + bouts["ext"] + bouts["latL"] + bouts["latR"] + tw_bouts)
    fr, fg, fe = fmt_score(flex_score)
    lr, lg, le = fmt_score(lat_score)
    hlr, hlg, hle = fmt_score(lat_held)
    hfr, hfg, hfe = fmt_score(flex_held)
    tgr, tgg, tge = fmt_score(tw_g_score)
    if len(flex_bouts) >= 10 and len(lat_bouts) >= 10:
        use = "main"
    elif len(flex_bouts) >= 10:
        use = "flex_only"
    else:
        use = "drop_insufficient"
    return {
        **row,
        "evidence_use": use,
        "n_flex_ext": len(bouts["flex"]) + len(bouts["ext"]),
        "n_lat": len(bouts["latL"]) + len(bouts["latR"]),
        "n_twist": len(tw_bouts),
        "bend_map": np.array2string(rmap, precision=3, suppress_small=True).replace("\n", " "),
        "insample_flex_r": fr, "insample_flex_gain": fg, "insample_flex_rmse": fe,
        "insample_lat_r": lr, "insample_lat_gain": lg, "insample_lat_rmse": le,
        "heldout_lat_from_flex_r": hlr,
        "heldout_lat_from_flex_abs_r": f"{abs(float(hlr)):.3f}" if hlr else "",
        "heldout_lat_from_flex_gain": hlg,
        "heldout_lat_from_flex_rmse": hle,
        "heldout_flex_from_lat_r": hfr,
        "heldout_flex_from_lat_abs_r": f"{abs(float(hfr)):.3f}" if hfr else "",
        "heldout_flex_from_lat_gain": hfg,
        "heldout_flex_from_lat_rmse": hfe,
        "twist_global_r": tgr, "twist_global_gain": tgg, "twist_global_rmse": tge,
        "bend_phi_drift_deg": f"{bend_phi[0]:.1f}",
        "bend_phi_range_deg": f"{bend_phi[1]:.1f}",
        "bend_phi_sd_deg": f"{bend_phi[2]:.1f}",
        "bend_phi_slope_deg_min": f"{bend_phi[3]:.2f}",
        "all_phi_drift_deg": f"{all_phi[0]:.1f}",
        "all_phi_range_deg": f"{all_phi[1]:.1f}",
        "all_phi_sd_deg": f"{all_phi[2]:.1f}",
        "all_phi_slope_deg_min": f"{all_phi[3]:.2f}",
    }


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    keys = list(rows[0])
    with path.open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)


def main() -> int:
    srows = sync_rows()
    write_csv(OUT / "batch_sync_summary.csv", srows)
    keep = [r for r in srows if str(r["class"]).startswith("clean") and r["subject"] not in {"03", "04"}]
    crows = []
    for r in keep:
        print(f"channel T{r['subject']} ({r['class']}, corr={float(r['corr']):.3f})", flush=True)
        crows.append(channel_row(r))
    write_csv(OUT / "batch_channel_summary.csv", crows)
    print(f"wrote {OUT / 'batch_sync_summary.csv'}")
    print(f"wrote {OUT / 'batch_channel_summary.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
