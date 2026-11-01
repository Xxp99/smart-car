"""Compile and exercise the real header-only obstacle implementation with MSVC."""
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
compiler = shutil.which("cl")
if not compiler:
    raise SystemExit("Run from an MSVC Developer Command Prompt")
with tempfile.TemporaryDirectory(prefix="smartcar_brick_") as folder:
    exe = Path(folder) / "test.exe"
    subprocess.run([compiler, "/nologo", "/std:c11", "/utf-8", "/W3",
                    str(root / "tests/test_brick_avoid.c"), f"/Fe:{exe}"], cwd=folder, check=True)
    subprocess.run([str(exe)], cwd=folder, check=True)
