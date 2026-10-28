"""Run the zebra regression tests with the real speed planner (MSVC Developer Prompt)."""
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
USER = ROOT / "镜头组例程" / "demo" / "user"

def main():
    compiler = shutil.which("cl")
    if not compiler:
        raise SystemExit("Run from an MSVC Developer Command Prompt")
    with tempfile.TemporaryDirectory(prefix="smartcar_zebra_") as directory:
        temp = Path(directory)
        header = (USER / "inc" / "pid.h").read_text(encoding="utf-8")
        stub = ('#include <stdint.h>\ntypedef uint8_t uint8;\n'
                'typedef uint16_t uint16;\ntypedef uint32_t uint32;\n'
                '#define MAX(a,b) ((a)>(b)?(a):(b))\n'
                '#define MIN(a,b) ((a)<(b)?(a):(b))\n'
                '#define func_abs(a) ((a)<0?-(a):(a))\n')
        (temp / "pid.h").write_text(header.replace('#include "zf_common_headfile.h"', stub), encoding="utf-8")
        subprocess.run([compiler, "/nologo", "/std:c11", "/utf-8", "/W3", f"/I{temp}",
                        str(USER / "src" / "pid.c"), str(ROOT / "tests" / "test_zebra_pass.c"),
                        f"/Fe:{temp / 'test.exe'}"], cwd=temp, check=True)
        subprocess.run([str(temp / "test.exe")], cwd=temp, check=True)

if __name__ == "__main__":
    main()
