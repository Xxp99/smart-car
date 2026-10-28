"""Exercise actual main-loop arbitration and motor ISR code with mocked hardware.

Run from an MSVC Developer Command Prompt. Fixtures do not replace track testing.
"""
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
USER = ROOT / "镜头组例程" / "demo" / "user"


def main(replay=None, output=None):
    compiler = shutil.which("cl")
    if not compiler:
        raise SystemExit("Run from an MSVC Developer Command Prompt")
    source = (USER / "src/main.c").read_text(encoding="utf-8")
    isr = (USER / "src/isr.c").read_text(encoding="utf-8")
    globals_code = source[source.index("uint8_t image["):source.index("int main(void)")]
    frame = source[source.index("            auxiliary_process("):
                   source.index("            for (uint8_t _i")]
    display = source[source.index("            char status_row[22];"):
                     source.index("            // 图像占用")]
    tick = isr[isr.index("        Speed_Target_Tick();"):
               isr.index("        pit_flag_clear(PIT_CH0);")]
    with tempfile.TemporaryDirectory(prefix="smartcar_race_") as directory:
        temp = Path(directory)
        stub = ('#include <stdint.h>\ntypedef uint8_t uint8;\n'
                'typedef uint16_t uint16;\ntypedef uint32_t uint32;\n'
                '#define MAX(a,b) ((a)>(b)?(a):(b))\n'
                '#define MIN(a,b) ((a)<(b)?(a):(b))\n'
                '#define func_abs(a) ((a)<0?-(a):(a))\n')
        for name in ("pid.h", "img_process.h", "main.h", "zebra_pass.h"):
            header = (USER / "inc" / name).read_text(encoding="utf-8")
            (temp / name).write_text(header.replace('#include "zf_common_headfile.h"', stub), encoding="utf-8")
        (temp / "frame_control.inc").write_text(
            globals_code + '\nstatic void control_frame(void) {\nunsigned char threshold=127;\n' +
            frame + display + '\n}\nstatic void motor_tick(void) {\n' + tick + '\n}\n', encoding="utf-8")
        defines = ["/DWIRELESS_IMAGE_ENABLE=0", "/DWIRELESS_SPEED_ENABLE=0"] + (["/DREPLAY_MODE"] if replay else [])
        subprocess.run([compiler, *defines, "/nologo", "/std:c11", "/utf-8", "/W3", f"/I{temp}",
                        str(USER / "src/img_process.c"), str(USER / "src/pid.c"),
                        str(ROOT / "tests/test_race_control.c"),
                        f"/Fe:{temp / 'test.exe'}"], cwd=temp, check=True)
        if replay:
            subprocess.run([str(temp / "test.exe"), str(Path(replay).resolve()),
                            str(Path(output).resolve())], cwd=temp, check=True)
        else:
            subprocess.run([str(temp / "test.exe")], cwd=temp, check=True)


if __name__ == "__main__":
    main()
