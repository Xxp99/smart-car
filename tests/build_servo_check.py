"""Build motor-off steering check and restore/build the normal firmware."""
from pathlib import Path
import subprocess,shutil
ROOT=Path(__file__).resolve().parents[1]
HEADER=ROOT/'镜头组例程/demo/user/inc/main.h'
PROJECT=ROOT/'镜头组例程/demo/mdk/rt1064.uvprojx'
AXF=PROJECT.parent/'Objects/rt1064.axf'
OUT=ROOT/'firmware'
def build(log):
    startup=subprocess.STARTUPINFO()
    startup.dwFlags|=subprocess.STARTF_USESHOWWINDOW
    startup.wShowWindow=0
    result=subprocess.run([r'D:\keil5\UV4\UV4.exe','-b',str(PROJECT),'-t','nor_sdram_zf_dtcm','-o',str(ROOT/log)],
        cwd=ROOT,startupinfo=startup,timeout=180)
    if result.returncode!=0:raise RuntimeError(f'Keil exit {result.returncode}: {log}')
def main():
    original=HEADER.read_bytes()
    OUT.mkdir(exist_ok=True)
    try:
        changed=original.replace(b'#define SERVO_TEST_ENABLE (0)',b'#define SERVO_TEST_ENABLE (1)').replace(
            b'#define SERVO_TEST_SWEEP (0)',b'#define SERVO_TEST_SWEEP (1)')
        assert changed!=original
        HEADER.write_bytes(changed)
        build('servo_check_build.log')
        shutil.copy2(AXF,OUT/'servo_check.axf')
    finally:
        HEADER.write_bytes(original)
        build('v2_build.log')
        shutil.copy2(AXF,OUT/'roundabout_v2.axf')
    print('Built servo_check.axf and roundabout_v2.axf; normal source settings restored.')
if __name__=='__main__':main()
