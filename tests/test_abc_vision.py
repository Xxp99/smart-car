"""Compile the real image-processing C source on Windows with MSVC and run fixtures.

Run: python tests/test_track_elements.py
Synthetic fixtures validate logic, not recognition accuracy on a real track.
"""
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
USER = ROOT / "镜头组例程" / "demo" / "user"
HARNESS = r'''
#include "img_process.h"
#include "pid.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
void display_tests(void);

static uint8_t left[120], right[120], mid[120], image[120 * 188];
static void road(void) {
    for (int y = 0; y < 120; ++y) { left[y] = 50; right[y] = 138; }
}
static void gap(int kind, int first, int last) {
    for (int y = first; y <= last; ++y) {
        if (kind & 1) left[y] = 0;
        if (kind & 2) right[y] = 187;
    }
}
static track_element_t classify(void) {
    return track_element_classify(left, right, 120, 188);
}
static track_nav_t nav;
static track_detector_t detector;
static uint8_t patched_left[120], patched_right[120], patched_mid[120];
static void navigate(unsigned now) {
    track_nav_update(&nav, &detector, left, right, NULL, patched_left, patched_right,
                     patched_mid, 120, 188, now);
    for (int y = 0; y < 120; ++y) {
        assert(patched_left[y] < 188 && patched_right[y] < 188 && patched_mid[y] < 188);
    }
}
static void learn(void) {
    memset(&nav, 0, sizeof nav); memset(&detector, 0, sizeof detector);
    road(); navigate(0); navigate(20);
    assert(nav.learned[96] && nav.width[96] == 88);
}
int main(void) {
    setvbuf(stdout, NULL, _IONBF, 0);
    // The real log showed UNKNOWN on a geometrically valid road. Width must
    // still be learned before a later ring candidate is confirmed.
    road(); memset(&nav,0,sizeof nav); memset(&detector,0,sizeof detector);
    detector.candidate = TRACK_UNKNOWN; navigate(20);
    assert(nav.learned[96] && nav.learned[72]);
    right[96] = 183; navigate(40);
    assert(nav.width[96] == 88); // The logged 133-pixel opening must not corrupt width.
    memset(&nav,0,sizeof nav); road(); right[96] = 183; navigate(60);
    assert(!nav.learned[96]);
    memset(&nav,0,sizeof nav); road();
    for (int y=90; y<=102; ++y) { left[y]=25; right[y]=158; }
    navigate(80);
    assert(nav.learned[96] && nav.width[96]==133); // Coherent wide near field is valid.
    memset(&nav,0,sizeof nav); memset(&detector,0,sizeof detector);
    road(); left[96]=0;
    nav.learned[72]=1; nav.width[72]=101;
    nav.vision.enabled=1; nav.vision.side[1].monotonic=1;
    detector.confirmed=detector.candidate=TRACK_ROUND_LEFT;
    navigate(100);
    assert(nav.phase==NAV_APPROACH && nav.fault==NAV_FAULT_NONE);
    assert(nav.learned[96] && nav.path_valid[96]);
    // Ring contour with B outside A (the logged false-positive geometry)
    // must not cause a ring confirmation, even when both points exist.
    for (int y=0;y<120;++y) for (int x=0;x<188;++x) {
        int border=188;
        if(y>=85) border=130;
        if(y>=35 && y<=65) border=12+2*(y>50?y-50:50-y);
        image[y*188+x]=x>45 && x<border?255:0;
    }
    auxiliary_process(image,120,188,127,left,mid,right);
    track_element_t bad=track_visual_classify(&nav.vision,image,left,right,120,188);
    assert(bad != TRACK_ROUND_RIGHT);
    ring_vision_t vision;
    for (int side = 0; side < 2; ++side) {
        for (int y = 0; y < 120; ++y) for (int x = 0; x < 188; ++x) {
            int xx = side ? x : 187-x;
            int border = 188;
            if (y >= 85) border = 140;
            if (y >= 35 && y <= 65) border = 120 + 2 * (y > 50 ? y-50 : 50-y);
            image[y*188+x] = xx > 45 && xx < border ? 255 : 0;
        }
        auxiliary_process(image,120,188,127,left,mid,right);
        track_element_t result = track_visual_classify(&vision,image,left,right,120,188);
        printf("visual side=%d result=%d A=%d B=%d C=%d outer=%d\n", side,result,
            vision.side[side].a.valid,vision.side[side].b.valid,vision.side[side].c.valid,
            vision.side[1-side].monotonic);
        assert(result == (side ? TRACK_ROUND_RIGHT : TRACK_ROUND_LEFT));
        learn();
        // learn() resets the row arrays, so restore actual image boundaries.
        auxiliary_process(image,120,188,127,left,mid,right);
        for (int frame = 0; frame < 3; ++frame) {
            abc_element_update(&detector, track_visual_classify(&nav.vision,image,left,right,120,188));
            navigate(100+frame*20);
        }
        assert(nav.phase == NAV_APPROACH);
        for (int y = 0; y < 120; ++y) for (int x = 0; x < 188; ++x) {
            int xx = side ? x : 187-x;
            int border = 188;
            if (y >= 60 && y <= 80) border = 120 + 2*(y>70?y-70:70-y);
            int roof = xx >= 80 && y <= 45 - (xx>110?xx-110:110-xx)/2;
            image[y*188+x] = xx>45 && xx<border && !roof ? 255 : 0;
        }
        auxiliary_process(image,120,188,127,left,mid,right);
        for (int frame = 0; frame < 3; ++frame) {
            abc_element_update(&detector, track_visual_classify(&nav.vision,image,left,right,120,188));
            navigate(200+frame*20);
        }
        printf("C-stage side=%d phase=%d A=%d B=%d C=%d cy=%d\n",side,nav.phase,
            nav.vision.side[side].a.valid,nav.vision.side[side].b.valid,
            nav.vision.side[side].c.valid,nav.vision.side[side].c.y);
        assert(nav.phase == NAV_ENTER);
        printf("steer side=%d raw=%d control=%d\n",side,mid[96],patched_mid[96]);
        assert(side ? patched_mid[96] > 94 : patched_mid[96] < 94);
        road();
        memset(&nav.vision.side,0,sizeof nav.vision.side);
        detector.candidate = detector.confirmed = TRACK_NONE;
        navigate(400); navigate(420); navigate(440);
        assert(nav.phase == NAV_INSIDE);
        // Without an exit corner, time and side loss alone cannot trigger exit.
        gap(side ? 1 : 2,80,100);
        navigate(1000); navigate(1020); navigate(1040);
        assert(nav.phase == NAV_INSIDE);
        nav.vision.side[1-side].a.valid = 1;
        nav.vision.side[1-side].a.x = side ? 50 : 138;
        nav.vision.side[1-side].a.y = 80;
        navigate(1100); navigate(1120); navigate(1140);
        assert(nav.phase == NAV_EXIT);
        road(); memset(&nav.vision.side,0,sizeof nav.vision.side);
        for (int t=1400;t<=1560;t+=40) navigate(t);
        assert(nav.phase == NAV_RECOVER);
        for (int t=1600;t<=2200;t+=40) navigate(t);
        assert(nav.phase == NAV_NORMAL);
    }
    learn();
    nav.vision.enabled=1;
    nav.vision.side[0].a.valid=1;
    nav.vision.side[0].a.x=5;
    nav.vision.side[0].a.y=97;
    detector.confirmed=detector.candidate=TRACK_ROUND_LEFT;
    navigate(2300); navigate(2500);
    assert(nav.phase==NAV_ENTER); // A reaches the camera bottom while C is absent.
    assert(patched_mid[96]<94); // Missing C still requests a left turn.
    road(); right[96]=187;
    navigate(2520);
    assert(nav.phase==NAV_ENTER && nav.path_valid[96] && patched_mid[96]<94);
    memset(image,255,sizeof image);
    auxiliary_process(image,120,188,127,left,mid,right);
    assert(track_visual_classify(&vision,image,left,right,120,188) == TRACK_UNKNOWN);
    memset(image,0,sizeof image);
    auxiliary_process(image,120,188,127,left,mid,right);
    assert(track_visual_classify(&vision,image,left,right,120,188) == TRACK_UNKNOWN);
    for (int fixture=0;fixture<3;++fixture) {
        for(int y=0;y<120;++y) for(int x=0;x<188;++x) {
            int shift=fixture==1 ? y/5 : 0;
            image[y*188+x]=(x>45+shift && x<140+shift) ? 255 : 0;
            if(fixture==2 && y>=85 && y<=100 && x>45) image[y*188+x]=255;
        }
        auxiliary_process(image,120,188,127,left,mid,right);
        track_element_t result=track_visual_classify(&vision,image,left,right,120,188);
        assert(result!=TRACK_ROUND_LEFT && result!=TRACK_ROUND_RIGHT);
    }
    puts("PASS: team ABC vision and lifecycle"); return 0;
}
'''


def main():
    compiler = shutil.which("cl")
    if not compiler:
        raise SystemExit("Run in an MSVC developer shell (cl.exe is required).")
    with tempfile.TemporaryDirectory(prefix="track-elements-") as folder:
        temp = Path(folder)
        header = (USER / "inc" / "img_process.h").read_text(encoding="utf-8")
        (temp / "img_process.h").write_text(
            header.replace('#include "zf_common_headfile.h"', '#include <stdint.h>'),
            encoding="utf-8",
        )
        pid_header = (USER / "inc" / "pid.h").read_text(encoding="utf-8")
        (temp / "pid.h").write_text(pid_header.replace('#include "zf_common_headfile.h"',
            '#include <stdint.h>\ntypedef uint8_t uint8;\ntypedef uint16_t uint16;\n'
            'typedef uint32_t uint32;\n#define MAX(a,b) ((a)>(b)?(a):(b))\n'
            '#define MIN(a,b) ((a)<(b)?(a):(b))\n#define func_abs(x) ((x)>=0?(x):-(x))'), encoding="utf-8")
        (temp / "test.c").write_text(HARNESS, encoding="utf-8")
        # Execute the actual glyph renderer with mocked SPI and inclusive screen bounds.
        driver = (ROOT / "镜头组例程/libraries/zf_device/zf_device_tft180.c").read_bytes().decode("latin1")
        start = driver.index("void tft180_show_char (")
        end = driver.index("void tft180_show_string (", start)
        renderer = driver[start:end]
        renderer = renderer[:renderer.rfind("}") + 1]
        mock = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
typedef uint8_t uint8;
typedef uint16_t uint16;
enum {TFT180_6X8_FONT, TFT180_8X16_FONT, TFT180_16X16_FONT};
static int tft180_display_font, expected, written;
static int tft180_x_max = 128, tft180_y_max = 160;
static uint16 tft180_pencolor, tft180_bgcolor;
static uint8 ascii_font_6x8[96][6], ascii_font_8x16[96][16];
#define zf_assert(x) assert(x)
static void chip_select(int level) {
    if (level) { assert(written == expected); expected = written = 0; }
}
#define TFT180_CS(x) chip_select(x)
static void tft180_set_region(int x1, int y1, int x2, int y2) {
    assert(written == expected);
    assert(0 <= x1 && x1 <= x2 && x2 < 128);
    assert(0 <= y1 && y1 <= y2 && y2 < 160);
    expected = (x2 - x1 + 1) * (y2 - y1 + 1); written = 0;
}
static void tft180_write_16bit_data(uint16 value) { (void)value; assert(++written <= expected); }
'''
        mock += renderer + r'''
void display_tests(void) {
    for (int ch = 32; ch <= 126; ++ch) {
        tft180_display_font = TFT180_6X8_FONT;
        tft180_show_char(0, 0, (char)ch);
        tft180_show_char(122, 152, (char)ch);
        tft180_display_font = TFT180_8X16_FONT;
        tft180_show_char(120, 144, (char)ch);
    }
    puts("PASS: actual TFT renderer, every printable glyph, bottom/right bounds, pixel counts");
}
'''
        (temp / "display_test.c").write_text(mock, encoding="utf-8")
        subprocess.run(
            [compiler, "/nologo", "/std:c11", "/utf-8", "/W3", f"/I{temp}",
             str(USER / "src" / "img_process.c"), str(USER / "src" / "pid.c"), str(temp / "test.c"),
             str(temp / "display_test.c"),
             f"/Fe:{temp / 'test.exe'}"], cwd=temp, check=True,
        )
        subprocess.run([str(temp / "test.exe")], cwd=temp, check=True)


if __name__ == "__main__":
    main()
