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
    track_nav_update(&nav, &detector, left, right, patched_left, patched_right,
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
static void mirror_road(void) {
    for (int y=0; y<120; ++y) {
        int l=left[y]; left[y]=187-right[y]; right[y]=187-l;
    }
}
static void partial_ring(int loss_first, int loss_last) {
    for (int y=0; y<120; ++y) { left[y]=90; right[y]=178; }
    gap(1,80,100);
    for (int y=loss_first; y<=loss_last; ++y) right[y]=187;
}
static int is_ring(track_element_t value) {
    return value==TRACK_ROUND_LEFT || value==TRACK_ROUND_RIGHT;
}
static void partial_ring_tests(void) {
    for (int mirror=0; mirror<2; ++mirror) {
        track_element_t expected=mirror ? TRACK_ROUND_RIGHT : TRACK_ROUND_LEFT;
        for (int length=1; length<=6; ++length) {
            partial_ring(94,93+length);
            if (mirror) mirror_road();
            assert(classify()==expected);
        }
        for (int coordinate=185; coordinate<=187; ++coordinate) {
            partial_ring(94,97);
            for (int y=94; y<=97; ++y) right[y]=(uint8_t)coordinate;
            if (mirror) mirror_road();
            assert(classify()==expected);
        }
        // Dropout in an anchor window: two actual anchor rows must still survive.
        partial_ring(94,97); right[78]=187;
        if (mirror) mirror_road();
        assert(classify()==expected);

        partial_ring(93,99); // Seven rows exceeds the spatial repair bound.
        if (mirror) mirror_road();
        assert(!is_ring(classify()));
        partial_ring(84,86); gap(2,93,96); // 7/23 lost overall despite short individual gaps.
        if (mirror) mirror_road();
        assert(!is_ring(classify()));
        partial_ring(94,97); right[90]=150; // Visible edge has a discontinuity.
        if (mirror) mirror_road();
        assert(!is_ring(classify()));
        partial_ring(94,97); right[92]=183; right[93]=178; // Moving away from border before loss.
        if (mirror) mirror_road();
        assert(!is_ring(classify()));
        partial_ring(94,97); right[96]=1; // Disappeared at the wrong side of the image.
        if (mirror) mirror_road();
        assert(!is_ring(classify()));
        partial_ring(94,97);
        for (int y=0; y<120; ++y) if (right[y]!=187) right[y]=138;
        if (mirror) mirror_road();
        assert(!is_ring(classify())); // Missing segment has no near-border support.
        partial_ring(80,100);
        if (mirror) mirror_road();
        assert(classify()==TRACK_CROSS); // Both edges missing together is not a ring.
        partial_ring(93,98);
        for (int y=0; y<120; ++y) if (left[y]) left[y]=28;
        if (mirror) mirror_road();
        assert(!is_ring(classify())); // Masked rows cannot inflate observed width past 120%.
        partial_ring(94,97);
        for (int y=70; y<80; ++y) left[y]=3+(79-y)*4;
        for (int y=101; y<111; ++y) left[y]=3+(y-101)*4;
        if (mirror) mirror_road();
        assert(!is_ring(classify())); // Gradual inner-edge clipping remains a curve.

        // Real detector + controller: prediction covers the control row without
        // corrupting the raw edges or immediately falling back to SLOW.
        learn(); partial_ring(94,97);
        if (mirror) mirror_road();
        uint8_t saved_l[120], saved_r[120];
        memcpy(saved_l,left,120); memcpy(saved_r,right,120);
        for (int frame=0; frame<ROUND_CONFIRM_FRAMES+NAV_FEATURE_FRAMES; ++frame) {
            track_element_update(&detector,classify());
            navigate(100+frame*20);
            if (frame<ROUND_CONFIRM_FRAMES-1) assert(nav.phase==NAV_NORMAL);
        }
        assert(nav.phase==NAV_ENTER && nav.path_valid[96]);
        int expected_mid=178-88/2-88*NAV_ENTRY_BIAS_PERCENT/100;
        assert(patched_mid[96]==(mirror ? 187-expected_mid : expected_mid));
        assert(memcmp(saved_l,left,120)==0 && memcmp(saved_r,right,120)==0);
        navigate(700); assert(nav.phase==NAV_ENTER); // Current-frame support remains usable.
        partial_ring(92,99);
        if (mirror) mirror_road();
        navigate(1000); assert(nav.phase==NAV_SLOW); // Long loss cannot be extrapolated.
    }
    // Smooth ordinary bends, including partial clipping on either side.
    for (int shift=-90; shift<=90; shift+=10)
        for (int bend=-80; bend<=80; bend+=10) {
            for (int y=0; y<120; ++y) {
                int center=94+shift+bend*(y-60)*(y-60)/3600;
                int l=center-44, r=center+44;
                left[y]=(uint8_t)(l<0?0:(l>186?186:l));
                right[y]=(uint8_t)(r>187?187:(r<1?1:r));
            }
            assert(!is_ring(classify()));
        }
    track_detector_t check={0};
    for (int i=0; i<ROUND_CONFIRM_FRAMES-1; ++i) track_element_update(&check,TRACK_ROUND_LEFT);
    assert(check.confirmed==TRACK_NONE);
    track_element_update(&check,TRACK_UNKNOWN);
    for (int i=0; i<ROUND_CONFIRM_FRAMES-1; ++i) track_element_update(&check,TRACK_ROUND_LEFT);
    assert(check.confirmed==TRACK_NONE);
    track_element_update(&check,TRACK_ROUND_LEFT);
    assert(check.confirmed==TRACK_ROUND_LEFT);
}
static void exit_and_far_road_tests(void) {
    for (int mirror=0; mirror<2; ++mirror) {
        learn(); nav.phase=NAV_INSIDE;
        nav.element=mirror?TRACK_ROUND_RIGHT:TRACK_ROUND_LEFT;
        nav.stage_ms=nav.started_ms=100;
        detector.candidate=TRACK_UNKNOWN;
        // Outlet is displaced right of the continuing island path (mirrored below).
        road();
        for(int y=30;y<80;++y) { left[y]=70; right[y]=158; }
        gap(2,80,100);
        if(mirror) mirror_road();
        navigate(700); navigate(720); navigate(740);
        assert(nav.phase==NAV_EXIT && nav.guide_valid && nav.drive_valid);
        assert(mirror ? patched_mid[96]<93 : patched_mid[96]>94);
        // It is NOT the old island-following center (94 / 93).
        assert(patched_mid[96] == (mirror?81:106));
        assert(left[96]==(mirror?0:50)); // Raw geometry remains intact.

        learn(); nav.phase=NAV_INSIDE;
        nav.element=mirror?TRACK_ROUND_RIGHT:TRACK_ROUND_LEFT;
        nav.stage_ms=nav.started_ms=100; detector.candidate=TRACK_UNKNOWN;
        road(); gap(2,80,100);
        // Gradual clipping, with no real mouth corner, must not trigger EXIT.
        right[79]=184; right[101]=184;
        if(mirror) mirror_road();
        navigate(700); navigate(720); navigate(740);
        assert(nav.phase==NAV_INSIDE);
    }
    learn(); nav.phase=NAV_CROSS; nav.started_ms=100;
    detector.candidate=TRACK_UNKNOWN;
    road();
    for(int y=30;y<70;++y) { left[y]=60; right[y]=148; }
    gap(3,70,119); navigate(100); navigate(400);
    assert(nav.phase==NAV_CROSS && nav.guide_valid && nav.drive_valid);
    assert(patched_mid[96]==104); // Current far lane, not old center 94.
    gap(3,30,119); navigate(450);
    assert(nav.drive_valid && patched_mid[96]==104);
    navigate(650);
    assert(nav.phase==NAV_SLOW && !nav.drive_valid); // Holding cannot renew its timestamp.
}
static void navigation_tests(void) {
    learn(); gap(3, 80, 99);
    detector.candidate = detector.confirmed = TRACK_CROSS;
    navigate(100);
    assert(nav.phase == NAV_CROSS);
    assert(patched_left[96] == 50 && patched_right[96] == 138 && patched_mid[96] == 94);
    // Missing lower anchors: retain incoming path briefly, then fall back to low-speed tracking.
    gap(3, 30, 119); navigate(150);
    assert(nav.phase == NAV_CROSS && patched_mid[96] == 94);
    navigate(301); assert(nav.phase == NAV_SLOW);
    road(); navigate(400); assert(nav.phase == NAV_SLOW);
    detector.candidate = detector.confirmed = TRACK_NONE;
    for (int t = 500; t <= 1100; t += 40) navigate(t);
    assert(nav.phase == NAV_NORMAL); // Automatically recover without reboot.

    learn(); gap(3, 80, 99); detector.candidate = detector.confirmed = TRACK_CROSS;
    navigate(100); road(); detector.candidate = detector.confirmed = TRACK_NONE;
    for (int t = 200; t <= 360; t += 40) navigate(t);
    assert(nav.phase == NAV_RECOVER);
    for (int t = 400; t <= 1000; t += 40) navigate(t);
    assert(nav.phase == NAV_NORMAL);

    for (int side = 1; side <= 2; ++side) {
        learn(); gap(side, 80, 100);
        detector.candidate = detector.confirmed = side == 1 ? TRACK_ROUND_LEFT : TRACK_ROUND_RIGHT;
        navigate(100); assert(nav.phase == NAV_APPROACH);
        navigate(120); navigate(140); assert(nav.phase == NAV_ENTER);
        assert(side == 1 ? patched_mid[96] < 94 : patched_mid[96] > 94);
        assert(nav.width[96] == 88); // Synthetic opening cannot overwrite learned road width.
        road(); detector.candidate = detector.confirmed = TRACK_NONE;
        navigate(300); navigate(320); navigate(340); assert(nav.phase == NAV_INSIDE);
        gap(side == 1 ? 2 : 1, 80, 100);
        detector.candidate = TRACK_UNKNOWN;
        navigate(400); navigate(420); navigate(440);
        assert(nav.phase == NAV_INSIDE); // No exit before minimum in-ring time.
        navigate(900); assert(nav.phase == NAV_EXIT);
        assert(patched_mid[96] == 94);
        road(); detector.candidate = TRACK_NONE;
        for (int t = 1100; t <= 1260; t += 40) navigate(t);
        assert(nav.phase == NAV_RECOVER);
        for (int t = 1300; t <= 1900; t += 40) navigate(t);
        assert(nav.phase == NAV_NORMAL);
    }
    learn(); gap(1, 80, 100); detector.candidate = detector.confirmed = TRACK_ROUND_LEFT;
    for (int y = 80; y <= 100; ++y) left[y] = ROUND_EDGE_MARGIN;
    navigate(100); navigate(120); navigate(140);
    assert(nav.phase == NAV_ENTER); // Recognition and entry use identical edge tolerance.
    learn(); gap(1, 80, 100); detector.candidate = detector.confirmed = TRACK_ROUND_LEFT;
    navigate(100); navigate(120); navigate(140);
    navigate(100 + NAV_TIMEOUT_MS); assert(nav.phase == NAV_SLOW);
    unsigned fallback_time = nav.stage_ms;
    navigate(100 + NAV_TIMEOUT_MS + 1000);
    assert(nav.phase == NAV_SLOW && nav.stage_ms == fallback_time); // No timeout reset loop.
    // No width calibration at startup: special element falls back to low-speed tracking.
    memset(&nav, 0, sizeof nav); navigate(0); assert(nav.phase == NAV_SLOW);
    // Unsigned elapsed time remains correct over the millisecond counter wrap.
    learn(); nav.phase = NAV_INSIDE; nav.element = TRACK_ROUND_LEFT;
    nav.started_ms = nav.stage_ms = 0xfffffff0u;
    road(); detector.candidate = detector.confirmed = TRACK_NONE;
    navigate(20); assert(nav.phase == NAV_INSIDE);
    // Recovery must not remain at low speed indefinitely when straight-line evidence never returns.
    learn(); nav.phase = NAV_RECOVER; nav.stage_ms = 100;
    for (int y = 0; y < 120; ++y) { left[y] += y / 2; right[y] += y / 3; }
    detector.candidate = TRACK_UNKNOWN;
    navigate(100 + NAV_RECOVER_TIMEOUT_MS); assert(nav.phase == NAV_SLOW);

    My_Pid_Init();
    Speed_Plan_Update(0, 0, 1, SPEED_CURVE_TARGET);
    Speed_Target_Tick(); assert(speed_target <= SPEED_CURVE_TARGET);
    Speed_Plan_Update(0, 0, 1, 0);
    Speed_Target_Tick(); assert(speed_target == 0);
    Speed_Plan_Update(0, 0, 1, SPEED_STRAIGHT_TARGET);
    for (int i = 0; i < 10; ++i) Speed_Target_Tick();
    assert(speed_target >= 10 * SPEED_ACCEL_STEP - 0.01f &&
           speed_target <= 10 * SPEED_ACCEL_STEP + 0.01f);
    for (int i = 0; i < SPEED_IMAGE_TIMEOUT_MS / SPEED_SAMPLE_PERIOD_MS; ++i) Speed_Target_Tick();
    assert(speed_target == 0);
}
int main(void) {
    road(); assert(classify() == TRACK_NONE);
    // A smooth ordinary curve retains both borders.
    for (int y = 0; y < 120; ++y) { left[y] += y / 5; right[y] += y / 5; }
    assert(classify() == TRACK_NONE);
    road(); gap(3, 55, 67); assert(classify() == TRACK_CROSS);
    road(); gap(1, 55, 67); assert(classify() == TRACK_ROUND_LEFT);
    road(); gap(2, 55, 67); assert(classify() == TRACK_ROUND_RIGHT);
    road(); gap(3, 55, 56); assert(classify() == TRACK_UNKNOWN);
    road(); gap(1, 30, 80); assert(classify() == TRACK_ROUND_LEFT); // 51 rows, previously rejected.
    road(); gap(1, 20, 100); assert(classify() == TRACK_UNKNOWN); // Still rejects broad side loss.
    for (int mirror = 0; mirror < 2; ++mirror) {
        track_element_t expected = mirror ? TRACK_ROUND_RIGHT : TRACK_ROUND_LEFT;
        for (int scenario = 0; scenario < 6; ++scenario) {
            road();
            if (scenario == 0) gap(1, 55, 58); // 4-row opening, previously minimum 6.
            if (scenario == 1) gap(1, 26, 40); // Upper anchors outside old ROI.
            if (scenario == 2) gap(1, 90, 110); // Lower anchors outside old ROI.
            if (scenario == 3) { // 28% widening, previously required 30%.
                for (int y = 0; y < 120; ++y) left[y] = 30;
                gap(1, 55, 67);
            }
            if (scenario == 4) { // Inner-border noise in one of three upper anchor rows.
                gap(1, 55, 67); left[53] = 0;
            }
            if (scenario == 5) { // Near-edge coordinates, not exactly zero.
                for (int y = 55; y <= 67; ++y) left[y] = 2;
            }
            if (mirror) for (int y = 0; y < 120; ++y) {
                int old_left = left[y]; left[y] = 187 - right[y]; right[y] = 187 - old_left;
            }
            assert(classify() == expected);
        }
        // Ordinary curve gradually leaves and re-enters the image: no abrupt opening corners.
        road(); gap(1, 55, 67);
        for (int y = 45; y < 55; ++y) left[y] = 3 + (54 - y) * 4;
        for (int y = 68; y < 78; ++y) left[y] = 3 + (y - 68) * 4;
        if (mirror) for (int y = 0; y < 120; ++y) {
            int old_left = left[y]; left[y] = 187 - right[y]; right[y] = 187 - old_left;
        }
        assert(classify() == TRACK_UNKNOWN);
    }
    road(); gap(1, 55, 56); assert(classify() == TRACK_UNKNOWN); // Tiny dropout remains rejected.
    road(); gap(1, 55, 119); assert(classify() == TRACK_UNKNOWN); // No lower anchor.
    road(); gap(1, 0, 67); assert(classify() == TRACK_UNKNOWN); // No upper anchor.
    road(); gap(3, 0, 119); assert(classify() == TRACK_UNKNOWN);
    road(); gap(1, 55, 67); right[60] = 160; assert(classify() == TRACK_UNKNOWN);
    road(); gap(1, 42, 49); gap(2, 75, 82); assert(classify() == TRACK_UNKNOWN);
    assert(track_element_classify(0, right, 120, 188) == TRACK_UNKNOWN);
    assert(track_element_classify(left, right, 10, 188) == TRACK_UNKNOWN);

    track_detector_t state = {0};
    track_element_update(&state, TRACK_CROSS);
    track_element_update(&state, TRACK_NONE);
    assert(state.confirmed == TRACK_NONE);
    for (int i = 0; i < ELEMENT_CONFIRM_FRAMES; ++i)
        track_element_update(&state, TRACK_CROSS);
    assert(state.confirmed == TRACK_CROSS);
    for (int i = 0; i < ELEMENT_RELEASE_FRAMES - 1; ++i)
        track_element_update(&state, TRACK_UNKNOWN);
    assert(state.confirmed == TRACK_CROSS);
    track_element_update(&state, TRACK_UNKNOWN);
    assert(state.confirmed == TRACK_NONE);
    for (int i = 0; i < 300; ++i) track_element_update(&state, TRACK_ROUND_RIGHT);
    assert(state.confirmed == TRACK_ROUND_RIGHT);
    assert(state.confirm_count == ROUND_CONFIRM_FRAMES);
    memset(&state, 0, sizeof state);
    for (int i = 0; i < 20; ++i)
        track_element_update(&state, i % 2 ? TRACK_ROUND_LEFT : TRACK_ROUND_RIGHT);
    assert(state.confirmed == TRACK_NONE); // Unstable directions cannot confirm a ring.

    // Real image-to-boundary pipeline with an Otsu threshold of zero.
    memset(image, 0, sizeof image);
    for (int y = 0; y < 120; ++y)
        for (int x = 51; x < 138; ++x) image[y * 188 + x] = 255;
    short histogram[256];
    memset(histogram, 0x33, sizeof histogram);
    get_hist_gram(image, 120, 188, histogram);
    assert(histogram[200] == 0 && histogram[255] == 120 * 87);
    unsigned char threshold = get_threshold_otsu(histogram);
    binaryzation_process(image, 120, 188, threshold);
    auxiliary_process(image, 120, 188, threshold, left, mid, right);
    assert(classify() == TRACK_NONE);
    assert(left[90] == 50 && right[90] == 138 && mid[90] == 94);
    // Widening to both sides creates the cross fixture before overlays are drawn.
    for (int y = 55; y <= 67; ++y) memset(image + y * 188, 255, 188);
    auxiliary_process(image, 120, 188, threshold, left, mid, right);
    assert(classify() == TRACK_CROSS);
    memset(image, 255, sizeof image);
    auxiliary_process(image, 120, 188, 0, left, mid, right);
    assert(classify() == TRACK_UNKNOWN);
    memset(image, 0, sizeof image);
    auxiliary_process(image, 120, 188, 0, left, mid, right);
    assert(classify() == TRACK_UNKNOWN);
    for (int i = 0; i < 120; ++i)
        assert(left[i] < 188 && mid[i] < 188 && right[i] < 188);
    partial_ring_tests();
    navigation_tests();
    exit_and_far_road_tests();
    display_tests();
    puts("PASS: recognition, partial outer clipping, 323 clipped bends rejected, five-frame confirmation, bounded control repair, binary pipeline, cross bridging, mirrored roundabout lifecycle, holds, timeout, speed limits");
    return 0;
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
