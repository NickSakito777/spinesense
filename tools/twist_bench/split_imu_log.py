from __future__ import annotations

"""Split a raw IMU log at device-timestamp resets without modifying the source.

The normal parser groups rows by timestamp. Feeding it a hard-merged/reset log can
therefore overwrite or reorder samples. Run this first, then fit one IMU/MoCap clock
per output segment with sync_audit.py.
"""

import argparse
import hashlib
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))


def data_row(line: str) -> tuple[float, str] | None:
    parts = line.strip().split()
    if len(parts) < 10 or not parts[1].upper().startswith("IMU"):
        return None
    try:
        return float(parts[0]), line.rstrip("\r\n")
    except ValueError:
        return None


def split_rows(text: str, min_drop_ms: float = 1000.0) -> list[list[tuple[float, str]]]:
    segments: list[list[tuple[float, str]]] = [[]]
    previous: float | None = None
    for line in text.splitlines():
        row = data_row(line)
        if row is None:
            continue
        timestamp, clean_line = row
        if previous is not None and timestamp < previous - min_drop_ms:
            segments.append([])
        segments[-1].append((timestamp, clean_line))
        previous = timestamp
    return [segment for segment in segments if segment]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def self_test() -> None:
    row = lambda t, imu: f"{t} {imu} x 0x32 0 0 1000 1 2 3 1 0 0 0"  # noqa: E731
    text = "\n".join([row(1000, "IMU0"), row(1000, "IMU1"), row(2000, "IMU0"), row(10, "IMU0")])
    segments = split_rows(text)
    assert [len(x) for x in segments] == [3, 1]
    assert segments[1][0][0] == 10
    import twist_bench_v0 as parser
    try:
        parser.parse_serial_text(text)
    except ValueError as exc:
        assert "timestamp reset" in str(exc)
    else:
        raise AssertionError("shared raw parser must reject reset logs")
    print("split_imu_log self-test: PASS")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path)
    parser.add_argument("--out-dir", type=Path)
    parser.add_argument("--prefix", help="Output prefix; defaults to input stem.")
    parser.add_argument("--min-drop-ms", type=float, default=1000.0)
    parser.add_argument("--force", action="store_true", help="Overwrite existing derived segment files.")
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.self_test:
        self_test()
        return 0
    if args.input is None or args.out_dir is None:
        raise SystemExit("--input and --out-dir are required (or use --self-test).")
    source = args.input.resolve()
    if not source.is_file():
        raise SystemExit(f"IMU log not found: {source}")

    segments = split_rows(source.read_text(encoding="utf-8", errors="replace"), args.min_drop_ms)
    if not segments:
        raise SystemExit(f"No IMU data rows parsed from {source}")

    args.out_dir.mkdir(parents=True, exist_ok=True)
    prefix = args.prefix or source.stem
    manifest = {
        "source": str(source),
        "source_sha256": sha256(source),
        "source_untouched": True,
        "reset_rule": f"new segment when timestamp drops by more than {args.min_drop_ms:g} ms",
        "segments": [],
    }
    for index, segment in enumerate(segments, 1):
        output = args.out_dir / f"{prefix}_segment{index:02d}.log"
        if output.exists() and not args.force:
            raise SystemExit(f"Refusing to overwrite {output}; pass --force for derived outputs.")
        output.write_text("\n".join(line for _, line in segment) + "\n", encoding="utf-8")
        manifest["segments"].append({
            "id": f"segment{index:02d}",
            "path": str(output),
            "rows": len(segment),
            "first_t_ms": segment[0][0],
            "last_t_ms": segment[-1][0],
        })

    manifest_path = args.out_dir / f"{prefix}_segments.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"wrote {len(segments)} segment(s) and {manifest_path}")
    if len(segments) == 1:
        print("note: no timestamp reset exceeded the configured threshold")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
