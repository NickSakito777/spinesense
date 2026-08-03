from __future__ import annotations

"""Build the current signed surface-marker reference from a Motive CSV.

Defaults match the newer JN/XP, C7/T2, L3/S1 marker convention. Override marker
names explicitly for a different capture; marker semantics must never be guessed
from correlation.
"""

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import sync_audit as sync


def unit(vector: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(vector, axis=-1, keepdims=True)
    return vector / np.where(norm < 1e-12, np.nan, norm)


def unwrap_deg(angle: np.ndarray) -> np.ndarray:
    return np.degrees(np.unwrap(np.radians(angle)))


def marker(markers: dict[str, np.ndarray], name: str) -> np.ndarray:
    candidates = (name, f"Trunk:{name}", f"Trunk 2:{name}")
    for candidate in candidates:
        if candidate in markers:
            return markers[candidate]
    raise KeyError(f"Missing marker {name!r}; tried {candidates}")


def pelvis_basis(markers: dict[str, np.ndarray], names: tuple[str, str, str, str]):
    left_front, right_front, left_back, right_back = (marker(markers, name) for name in names)
    mediolateral = unit((right_front + right_back) / 2.0 - (left_front + left_back) / 2.0)
    anterior = (left_front + right_front) / 2.0 - (left_back + right_back) / 2.0
    up = unit(np.cross(mediolateral, anterior))
    if float(np.nanmedian(up[:, 1])) < 0.0:  # Motive global Y is up
        up = -up
    forward = unit(np.cross(up, mediolateral))
    return forward, up, mediolateral


def chord_tilt(parent, child, forward, up, mediolateral, still):
    chord = child - parent
    flex = unwrap_deg(np.degrees(np.arctan2(np.sum(chord * forward, axis=1), np.sum(chord * up, axis=1))))
    lateral = unwrap_deg(np.degrees(np.arctan2(np.sum(chord * mediolateral, axis=1), np.sum(chord * up, axis=1))))
    return flex - np.median(flex[still]), lateral - np.median(lateral[still])


def axial_angle(anterior, posterior, forward, mediolateral, still):
    direction = anterior - posterior
    angle = unwrap_deg(np.degrees(np.arctan2(
        np.sum(direction * mediolateral, axis=1),
        np.sum(direction * forward, axis=1),
    )))
    return angle - np.median(angle[still])


def mean_markers(markers: dict[str, np.ndarray], names: list[str]) -> np.ndarray:
    return np.mean([marker(markers, name) for name in names], axis=0)


def self_test() -> None:
    angle = np.radians(np.array([0.0, 10.0, -8.0]))
    forward = np.tile([0.0, 0.0, 1.0], (3, 1))
    up = np.tile([0.0, 1.0, 0.0], (3, 1))
    ml = np.tile([1.0, 0.0, 0.0], (3, 1))
    origin = np.zeros((3, 3))
    flex_child = np.column_stack([np.zeros(3), np.cos(angle), np.sin(angle)])
    flex, lateral = chord_tilt(origin, flex_child, forward, up, ml, np.array([True, False, False]))
    axial = axial_angle(np.column_stack([np.sin(angle), np.zeros(3), np.cos(angle)]), origin, forward, ml,
                        np.array([True, False, False]))
    assert np.allclose(flex, [0.0, 10.0, -8.0])
    assert np.allclose(lateral, 0.0)
    assert np.allclose(axial, [0.0, 10.0, -8.0])
    print("mocap_signed_reference self-test: PASS")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--report-json", type=Path)
    parser.add_argument("--neutral-seconds", type=float, default=8.0)
    parser.add_argument("--pelvis-left-front", default="LPSIS_F")
    parser.add_argument("--pelvis-right-front", default="RPSIS_F")
    parser.add_argument("--pelvis-left-back", default="LPSIS_B")
    parser.add_argument("--pelvis-right-back", default="RPSIS_B")
    parser.add_argument("--upper", default="C7 (2),T2", help="Comma-separated markers averaged as upper point.")
    parser.add_argument("--sternum", default="JN,XP", help="Comma-separated markers averaged as sternum point.")
    parser.add_argument("--sacrum", default="L3,S1,LPSIS_B,RPSIS_B", help="Comma-separated markers averaged as sacrum point.")
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args()


def names(value: str) -> list[str]:
    return [part.strip() for part in value.split(",") if part.strip()]


def main() -> int:
    args = parse_args()
    if args.self_test:
        self_test()
        return 0
    if args.input is None or args.output is None:
        raise SystemExit("--input and --output are required (or use --self-test).")

    time_s, markers = sync.parse_motive_markers(args.input, gap_fill=True)
    pelvis_names = (
        args.pelvis_left_front, args.pelvis_right_front,
        args.pelvis_left_back, args.pelvis_right_back,
    )
    try:
        forward, up, mediolateral = pelvis_basis(markers, pelvis_names)
        upper = mean_markers(markers, names(args.upper))
        sternum = mean_markers(markers, names(args.sternum))
        sacrum = mean_markers(markers, names(args.sacrum))
    except KeyError as exc:
        raise SystemExit(str(exc)) from exc

    still = time_s <= args.neutral_seconds
    if int(np.count_nonzero(still)) < 5:
        raise SystemExit("Neutral window has fewer than five MoCap frames")
    flex, lateral = chord_tilt(sacrum, upper, forward, up, mediolateral, still)
    axial = axial_angle(sternum, sacrum, forward, mediolateral, still)
    lateral_dominant = (np.abs(lateral) > 15.0) & (np.abs(flex) < 10.0)
    leakage_gain = None
    if int(np.count_nonzero(lateral_dominant)) > 50:
        leakage_gain = float(np.polyfit(lateral[lateral_dominant], axial[lateral_dominant], 1)[0])
        axial = axial - leakage_gain * lateral

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(["time_s", "flex_deg", "lateral_deg", "axial_deg"])
        writer.writerows(zip(time_s, flex, lateral, axial))

    report = {
        "input": str(args.input),
        "output": str(args.output),
        "neutral_seconds": args.neutral_seconds,
        "pelvis_markers": pelvis_names,
        "upper_markers": names(args.upper),
        "sternum_markers": names(args.sternum),
        "sacrum_markers": names(args.sacrum),
        "lateral_to_axial_leakage_gain": leakage_gain,
        "claim_boundary": "surface-marker proxy angles; not vertebral or Cobb angles",
    }
    report_path = args.report_json or args.output.with_suffix(".json")
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"wrote {args.output} and {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
