"""Host-test the real ASCII speed/state transport.

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
    with tempfile.TemporaryDirectory(prefix="smartcar_speed_") as directory:
        temp = Path(directory)
        subprocess.run([compiler, "/nologo", "/std:c11", "/utf-8", "/W4",
                        f"/I{temp}", f"/I{USER / 'inc'}",
                        str(ROOT / "tests/test_wireless_speed.c"),
                        f"/Fe:{temp / 'test.exe'}"], cwd=temp, check=True)
        subprocess.run([str(temp / "test.exe")], cwd=temp, check=True)


if __name__ == "__main__":
    main()
