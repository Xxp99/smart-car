"""Host-test the real image transport against the project's Seekfree serializer.

Run in an MSVC Developer Command Prompt. Hardware timing still needs testing.
"""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
USER = ROOT / "镜头组例程/demo/user"
LIB = ROOT / "镜头组例程/libraries/zf_device"


def function(source, name):
    start = source.index("void " + name + " (")
    brace = source.index("{", start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def main():
    compiler = shutil.which("cl")
    if not compiler:
        raise SystemExit("Run from an MSVC Developer Command Prompt")
    source = (LIB / "zf_device_detector.c").read_text(encoding="gb18030")
    header = (LIB / "zf_device_detector.h").read_text(encoding="gb18030")
    header = re.sub(r'^\s*#include[^\n]*', '', header, flags=re.M)
    reference = '\n'.join(function(source, name) for name in (
        "detector_camera_data_send", "detector_camera_dot_send",
        "detector_camera_information_config", "detector_camera_boundary_config",
        "detector_camera_send"))
    with tempfile.TemporaryDirectory(prefix="smartcar_image_") as directory:
        temp = Path(directory)
        (temp / "reference.h").write_text(header, encoding="utf-8")
        (temp / "reference.inc").write_text(reference, encoding="utf-8")
        subprocess.run([compiler, "/nologo", "/std:c11", "/utf-8", "/W4",
                        f"/I{temp}", f"/I{USER / 'inc'}",
                        str(ROOT / "tests/test_wireless_image.c"),
                        f"/Fe:{temp / 'test.exe'}"], cwd=temp, check=True)
        subprocess.run([str(temp / "test.exe")], cwd=temp, check=True)


if __name__ == "__main__":
    main()
