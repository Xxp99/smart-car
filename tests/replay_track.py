"""Replay captured grayscale frames through the real C main-loop control block.

MSVC developer prompt:
  python tests/replay_track.py capture.csv --output replay.csv

Manifest columns: ms,file. Files are headerless 188x120 uint8 grayscale,
before binarization/overlays, relative to the manifest. Time starts at zero,
strictly increases and is quantized to 10ms (the firmware's control clock).
Output target is the smoothed target at image arrival, before the next tick.
No vehicle model or actual wheel feedback is simulated.
"""
import argparse
import csv
from pathlib import Path
import struct
import tempfile

from test_race_control import main as run_control


def pack(manifest, destination):
    previous = -1
    with manifest.open(encoding="utf-8-sig", newline="") as source, destination.open("wb") as out:
        reader = csv.DictReader(source)
        if not {"ms", "file"}.issubset(reader.fieldnames or []):
            raise ValueError("Manifest needs ms,file columns")
        for index, row in enumerate(reader):
            timestamp = int(row["ms"])
            if (timestamp <= previous or timestamp % 10 or timestamp > 0xffffffff
                    or (index == 0 and timestamp != 0)):
                raise ValueError("Times must start at 0 and increase in 10ms units")
            pixels = (manifest.parent / row["file"]).read_bytes()
            if len(pixels) != 188 * 120:
                raise ValueError(f"Wrong image size: {row['file']}")
            out.write(struct.pack("<I", timestamp))
            out.write(pixels)
            previous = timestamp
    if previous < 0:
        raise ValueError("Empty capture")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Choose a new output path; existing results are preserved")
    with tempfile.TemporaryDirectory(prefix="smartcar_replay_") as folder:
        packed = Path(folder) / "frames.bin"
        pack(args.manifest.resolve(), packed)
        run_control(packed, args.output)
    print(f"Replay written: {args.output.resolve()}")


if __name__ == "__main__":
    main()
