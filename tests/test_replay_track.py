"""Smoke-test timestamped replay, Otsu input, and finish latch at three frame rates."""
import csv
from pathlib import Path
import tempfile

from replay_track import pack, run_control


def main():
    with tempfile.TemporaryDirectory(prefix="smartcar_capture_test_") as folder:
        root = Path(folder)
        road = bytearray(188 * 120)
        for y in range(120):
            road[y*188+51:y*188+138] = bytes([200]) * 87
        stripes = bytearray(road)
        for y in range(60, 90):
            for x in range(40, 148):
                stripes[y*188+x] = 200 if (x//6) % 2 else 0
        (root / "road.raw").write_bytes(road)
        (root / "stripes.raw").write_bytes(stripes)
        for period in (20, 50, 100):
            manifest = root / "capture.csv"
            with manifest.open("w", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(["ms", "file"])
                for ms in range(0, 5600, period):
                    writer.writerow([ms, "stripes.raw" if 3000 <= ms < 3400 else "road.raw"])
            packed, output = root / "frames.bin", root / "replay.csv"
            pack(manifest, packed)
            run_control(packed, output)
            with output.open(newline="") as file:
                records = list(csv.DictReader(file))
            assert any("ARMED" in r["zebra"] for r in records)
            assert any("Z:PASS" in r["zebra"] for r in records)
            assert any("Z:CLEAR" in r["zebra"] for r in records)
            assert all("STOP" not in r["zebra"] for r in records if 3000 <= int(r["ms"]) < 3400)
            assert all("STOP" in r["zebra"] and float(r["target"]) == 0
                       for r in records if int(r["ms"]) >= 5000)
            assert all(r["near_mid"] == "94" for r in records if int(r["ms"]) < 3000)
        print("PASS: real grayscale replay at 50/20/10 fps, arming and persistent finish stop")


if __name__ == "__main__":
    main()
