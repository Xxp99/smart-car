#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "pid.h"
#include "../镜头组例程/demo/user/inc/zebra_pass.h"
static uint8_t img[120][188];

static void race_frame(zebra_race_t *race, zebra_pass_t *pass, int candidate, uint32_t now)
{
    zebra_update(pass, (uint8_t)candidate, !candidate, 94, 94, now);
    zebra_race_update(race, pass, (uint8_t)candidate, now, now * 3u);
    zebra_steering(pass, 90, now);
}

static void race_tests(void)
{
    zebra_race_t race = {0};
    zebra_pass_t pass = {0};
    // Startup line persists longer than the guard: it must NEVER become a finish.
    for (uint32_t t=10000; t<=20000; t+=20)
    {
        race_frame(&race, &pass, 1, t);
        assert(race.phase == ZEBRA_RACE_START);
    }
    // Briefly absent then visible again: still the departure line.
    for (uint32_t t=20020; t<=20600; t+=20) race_frame(&race, &pass, 0, t);
    assert(race.phase == ZEBRA_RACE_START);
    for (uint32_t t=20620; t<=20800; t+=20) race_frame(&race, &pass, 1, t);
    assert(race.phase == ZEBRA_RACE_START);
    for (uint32_t t=20820; t<=24000; t+=20) race_frame(&race, &pass, 0, t);
    assert(race.phase == ZEBRA_RACE_ARMED);
    race_frame(&race, &pass, 1, 24020);
    race_frame(&race, &pass, 1, 24040);
    assert(race.phase == ZEBRA_RACE_ARMED);
    race_frame(&race, &pass, 0, 24060); // Isolated detections do not accumulate.
    race_frame(&race, &pass, 1, 24080);
    race_frame(&race, &pass, 1, 24100);
    race_frame(&race, &pass, 1, 24400); // Camera pause breaks finish confirmation.
    assert(race.phase == ZEBRA_RACE_ARMED && race.finish_hits == 1);
    race_frame(&race, &pass, 1, 24420);
    race_frame(&race, &pass, 1, 24440);
    assert(race.phase == ZEBRA_RACE_PASS);
    for (uint32_t t=24460; t<=28000; t+=20) race_frame(&race, &pass, 0, t);
    assert(race.phase == ZEBRA_RACE_STOP);

    // Starting just beyond the line: no permanent wait for a missed first stripe.
    // Also verify time differences across uint32 rollover.
    memset(&race, 0, sizeof(race)); memset(&pass, 0, sizeof(pass));
    uint32_t base = UINT32_MAX - 1000;
    for (uint32_t dt=0; dt<ZEBRA_START_GUARD_MS; dt+=20)
    {
        race_frame(&race, &pass, 0, base+dt);
        assert(race.phase == ZEBRA_RACE_START);
    }
    race_frame(&race, &pass, 0, base+ZEBRA_START_GUARD_MS);
    assert(race.phase == ZEBRA_RACE_ARMED);

    memset(&race, 0, sizeof(race)); memset(&pass, 0, sizeof(pass));
    for (uint32_t t=0; t<=600; t+=20) race_frame(&race, &pass, 0, t);
    race_frame(&race, &pass, 0, 2200); // Stale camera cannot stand for a clear road.
    assert(race.phase == ZEBRA_RACE_START);
    for (uint32_t t=2220; t<3000; t+=20) race_frame(&race, &pass, 0, t);
    assert(race.phase == ZEBRA_RACE_START);
    race_frame(&race, &pass, 0, 3000);
    assert(race.phase == ZEBRA_RACE_START); // Rebuild five clear frames after the gap.
    for (uint32_t t=3020; t<=3100; t+=20) race_frame(&race, &pass, 0, t);
    assert(race.phase == ZEBRA_RACE_ARMED);
    memset(&pass,0,sizeof pass);
    zebra_update(&pass,1,0,94,94,0);
    zebra_update(&pass,1,0,94,94,20);
    zebra_update(&pass,1,0,94,94,500);
    assert(pass.phase==ZEBRA_NORMAL && pass.hits==1);
}

static void finish_clearance_tests(void)
{
    zebra_race_t race = {0};
    zebra_pass_t pass = {0};
    race.phase = ZEBRA_RACE_ARMED;
    for (uint32_t t=0;t<=40;t+=20) race_frame(&race,&pass,1,t);
    assert(race.phase==ZEBRA_RACE_PASS);
    // Persistent stripes never trigger the finish latch, even after GO90 times out.
    for (uint32_t t=60;t<2000;t+=20) race_frame(&race,&pass,1,t);
    assert(race.phase==ZEBRA_RACE_PASS);
    // No stripe + invalid road is not a confirmed exit.
    for (uint32_t t=2000;t<2200;t+=20) {
        zebra_update(&pass,0,0,94,94,t);
        zebra_race_update(&race,&pass,0,t,10000);
    }
    assert(race.phase==ZEBRA_RACE_PASS);
    // Recovered lane starts a fresh distance measurement.
    for (uint32_t t=2200;t<=2400;t+=20) {
        zebra_update(&pass,0,1,94,94,t);
        zebra_race_update(&race,&pass,0,t,10000);
    }
    assert(race.phase==ZEBRA_RACE_CLEAR && race.clearance_start_counts==10000);
    for (uint32_t t=2420;t<=2800;t+=20) {
        zebra_update(&pass,0,1,94,94,t);
        zebra_race_update(&race,&pass,0,t,10000);
    }
    assert(race.phase==ZEBRA_RACE_CLEAR); // Stationary wheels: no time-based finish.
    zebra_race_update(&race,&pass,0,2820,9990);
    assert(race.phase==ZEBRA_RACE_CLEAR); // Reverse cannot wrap to a huge positive distance.
    zebra_race_update(&race,&pass,0,2840,10000+ZEBRA_FINISH_CLEAR_COUNTS-1);
    assert(race.phase==ZEBRA_RACE_CLEAR);
    // Reappearing stripe cancels old clearance, not just its display.
    zebra_update(&pass,1,0,94,94,2860);
    zebra_race_update(&race,&pass,1,2860,12000);
    assert(race.phase==ZEBRA_RACE_PASS);
    for (uint32_t t=2880;t<=3100;t+=20) {
        zebra_update(&pass,0,1,94,94,t);
        zebra_race_update(&race,&pass,0,t,12000);
    }
    assert(race.phase==ZEBRA_RACE_CLEAR && race.clearance_start_counts==12000);
    // A stale frame must discard progress and rebuild visual evidence.
    zebra_update(&pass,0,1,94,94,3500);
    zebra_race_update(&race,&pass,0,3500,20000);
    assert(race.phase==ZEBRA_RACE_PASS);
    for (uint32_t t=3520;t<=3700;t+=20) {
        zebra_update(&pass,0,1,94,94,t);
        zebra_race_update(&race,&pass,0,t,UINT32_MAX-100);
    }
    assert(race.phase==ZEBRA_RACE_CLEAR);
    uint32_t finish = (UINT32_MAX-100)+ZEBRA_FINISH_CLEAR_COUNTS;
    zebra_update(&pass,0,1,94,94,3720);
    zebra_race_update(&race,&pass,0,3720,finish);
    assert(race.phase==ZEBRA_RACE_STOP); // Distance wrap is supported.
    zebra_race_update(&race,&pass,1,3740,0);
    assert(race.phase==ZEBRA_RACE_STOP);

    speed_distance_counts=0;
    Speed_Odom_Update(20,40); assert(speed_distance_counts==30);
    Speed_Odom_Update(0,0); assert(speed_distance_counts==30);
    Speed_Odom_Update(-20,-40); assert(speed_distance_counts==0);
    speed_distance_counts=UINT32_MAX-10;
    Speed_Odom_Update(20,20); assert(speed_distance_counts==9);
}

int main(void)
{
    race_tests();
    finish_clearance_tests();
    memset(img, 255, sizeof(img));
    assert(!zebra_detect(&img[0][0], 120, 188));
    memset(img, 0, sizeof(img));
    assert(!zebra_detect(&img[0][0], 120, 188));
    /* Straight and curved white road between two black sides. */
    for (int bend = 0; bend < 2; ++bend)
    {
        memset(img, 0, sizeof(img));
        for (int y = 0; y < 120; ++y)
            for (int x = 35 + bend*y/4; x < 140 + bend*y/4; ++x) img[y][x] = 255;
        assert(!zebra_detect(&img[0][0], 120, 188));
    }
    /* Single transverse bar is not a zebra. */
    memset(img, 255, sizeof(img));
    memset(img[70], 0, 188*8);
    assert(!zebra_detect(&img[0][0], 120, 188));
    for (int y = 60; y < 90; ++y)
        for (int x = 35; x < 150; ++x) img[y][x] = (x/6)%2 ? 255 : 0;
    assert(zebra_detect(&img[0][0], 120, 188));
    memset(img, 255, sizeof(img));
    for (int y = 40; y < 100; ++y)
        for (int x = 40; x < 148; ++x) img[y][x] = (y/5)%2 ? 255 : 0;
    assert(zebra_detect(&img[0][0], 120, 188));
    /* Regression: single-pixel black OR white streaks must not form stripes. */
    for (int inverse = 0; inverse < 2; ++inverse)
    {
        memset(img, inverse ? 0 : 255, sizeof(img));
        for (int y=70; y<73; ++y)
            for (int x=40; x<=100; x+=10) img[y][x] = inverse ? 255 : 0;
        assert(!zebra_detect(&img[0][0], 120, 188));
    }
    for (int shift=-60; shift<=60; shift+=10)
        for (int bend=-80; bend<=80; bend+=10)
        {
            memset(img, 0, sizeof(img));
            for (int y=0; y<120; ++y)
            {
                int center=94+shift+bend*y*y/14400, half=20+y/4;
                for(int x=0; x<188; ++x)
                    if(x>center-half && x<center+half) img[y][x]=255;
            }
            assert(!zebra_detect(&img[0][0], 120, 188));
        }

    zebra_pass_t s = {0};
    zebra_update(&s, 1, 0, 40, 60, 0);
    zebra_update(&s, 1, 0, 40, 60, 20);
    assert(s.phase == ZEBRA_NORMAL && zebra_slow(&s, 1));
    assert(zebra_steering(&s, 108, 20) == 108); // Candidate does not take steering.
    zebra_update(&s, 0, 1, 94, 94, 40); // Break consecutive confirmations.
    zebra_update(&s, 1, 0, 40, 60, 60);
    zebra_update(&s, 1, 0, 40, 60, 80);
    assert(s.phase == ZEBRA_NORMAL);
    zebra_update(&s, 1, 0, 40, 60, 100);
    assert(s.phase == ZEBRA_STRAIGHT);
    assert(zebra_steering(&s, 72, 100) == 90);
    assert(zebra_steering(&s, 108, 100) == 90);
    // Disappearance alone cannot release steering while borders remain lost.
    for (int t=120; t<=300; t+=20) zebra_update(&s, 0, 0, 94, 94, t);
    assert(s.phase == ZEBRA_STRAIGHT);
    // Alternating false centers also cannot release it.
    for (int i=0; i<6; ++i) zebra_update(&s, 0, 1, i%2 ? 94 : 120, 94, 320+i*20);
    assert(s.phase == ZEBRA_STRAIGHT);
    for (int t=440; t<=520; t+=20) zebra_update(&s, 0, 1, 94, 94, t);
    assert(s.phase == ZEBRA_RECOVER);
    uint32_t release = s.phase_ms;
    assert(zebra_steering(&s, 90, release) == 90);
    // Reproduce a bad frame near the exit: 90->72 target must not cause a kick.
    zebra_update(&s, 0, 1, 130, 94, release+50);
    assert(zebra_steering(&s, 72, release+50) == 90);
    zebra_update(&s, 0, 1, 94, 94, release+100);
    assert(zebra_steering(&s, 90, release+100) == 90);
    zebra_update(&s, 0, 1, 94, 94, release+150);
    assert(zebra_steering(&s, 90, release+150) == 90);
    // A remaining zebra stripe during BACK cannot inject a new steering target.
    zebra_update(&s, 1, 0, 130, 94, release+200);
    assert(zebra_steering(&s, 72, release+200) == 90);
    // A real, persistent turn is accepted; do not accelerate until the output catches up.
    float previous = 90;
    for (unsigned dt=250; dt<=1000; dt+=50)
    {
        zebra_update(&s, 0, 1, 74, 74, release+dt);
        float angle = zebra_steering(&s, 100, release+dt);
        assert(fabsf(angle-previous) <= 1.001f);
        if (angle < 99.95f) assert(s.phase == ZEBRA_RECOVER && zebra_slow(&s, 0));
        previous = angle;
    }
    assert(s.phase == ZEBRA_NORMAL && !zebra_slow(&s, 0));
    assert(previous == 100);

    // Limiter is independent of frame rate, tolerates a long frame, and handles rollover.
    for (int direction=-1; direction<=1; direction+=2)
    {
        memset(&s, 0, sizeof(s));
        s.phase=ZEBRA_TIMEOUT;
        s.steering_angle=s.steering_target=90;
        s.steering_ms=UINT32_MAX-20;
        float target=90+direction*18.0f;
        assert(fabsf(zebra_steering(&s,target,29)-(90+direction)) < 0.01f);
        assert(fabsf(zebra_steering(&s,target,39)-(90+direction*1.2f)) < 0.01f);
        assert(fabsf(zebra_steering(&s,target,1039)-(90+direction*2.2f)) < 0.01f);
    }

    // Recovery cannot retain an old steering target forever after persistent lost lines.
    memset(&s, 0, sizeof(s));s.phase=ZEBRA_RECOVER;s.phase_ms=0;
    s.steering_angle=s.steering_target=90;
    zebra_update(&s,0,0,40,40,ZEBRA_STRAIGHT_MAX_MS);
    assert(s.phase == ZEBRA_TIMEOUT && zebra_slow(&s,0));

    // Confirm even with occluded lines; bounded hold and no immediate retrigger.
    memset(&s, 0, sizeof(s));
    for (int t=0; t<=40; t+=20) zebra_update(&s, 1, 0, 40, 60, t);
    zebra_update(&s, 1, 0, 40, 60, 40+ZEBRA_STRAIGHT_MAX_MS-1);
    assert(s.phase == ZEBRA_STRAIGHT);
    zebra_update(&s, 1, 0, 40, 60, 40+ZEBRA_STRAIGHT_MAX_MS);
    assert(s.phase == ZEBRA_TIMEOUT);
    assert(zebra_slow(&s, 0));
    assert(zebra_steering(&s, 108, 40+ZEBRA_STRAIGHT_MAX_MS) == 90);
    for (int t=1590; t<3000; t+=50)
    {
        zebra_update(&s, 1, 0, 40, 60, t);
        zebra_steering(&s,108,t);
    }
    assert(s.phase == ZEBRA_TIMEOUT);
    assert(s.steering_angle == 108);
    for (int t=3000; t<=4300; t+=20)
    {
        zebra_update(&s, 0, 1, 94, 94, t);
        zebra_steering(&s,90,t);
    }
    assert(s.phase == ZEBRA_NORMAL);
    // Re-arm for a subsequent zebra after normal line recovery.
    for (int t=4340; t<=4380; t+=20) zebra_update(&s, 1, 0, 40, 60, t);
    assert(s.phase == ZEBRA_STRAIGHT);
    s.phase_ms = UINT32_MAX-100;
    zebra_update(&s, 1, 0, 40, 60, (uint32_t)(s.phase_ms+ZEBRA_STRAIGHT_MAX_MS));
    assert(s.phase == ZEBRA_TIMEOUT);

    // Every new status line fits the portrait LCD and clears previous text.
    for (int phase=ZEBRA_NORMAL; phase<=ZEBRA_TIMEOUT; ++phase)
    {
        s.phase = (zebra_phase_t)phase;
        for (int race_phase=ZEBRA_RACE_START; race_phase<=ZEBRA_RACE_STOP; ++race_phase)
        for (int candidate=0; candidate<2; ++candidate)
        {
            zebra_race_t race = {0};
            race.phase = (zebra_race_phase_t)race_phase;
            size_t len = strlen(zebra_status(&s, &race, candidate));
            assert(len == 21 && len*6 <= 128 && 92+7 < 160);
        }
    }
    uint8_t l[120], r[120], bad[120];
    memset(l, 40, sizeof(l)); memset(r, 140, sizeof(r)); memset(bad, 1, sizeof(bad));
    assert(!zebra_row_valid(l,r,bad,96,188));
    bad[96] = 0; assert(zebra_row_valid(l,r,bad,96,188));
    r[96] = 42; assert(!zebra_row_valid(l,r,bad,96,188));

    // Actual speed planner: slow through zebra, recover on straight, stay slow in bend.
    speed_target = SPEED_STRAIGHT_TARGET;
    for (int i=0; i<(int)((SPEED_STRAIGHT_TARGET-ZEBRA_PASS_SPEED)/SPEED_DECEL_STEP)+2; ++i)
    {
        Speed_Plan_Update(0, 0, 1, ZEBRA_PASS_SPEED);
        Speed_Target_Tick();
        assert(speed_target > 0);
    }
    assert(fabsf(speed_target-ZEBRA_PASS_SPEED) < 0.01f);
    for (int i=0; i<250; ++i)
    {
        Speed_Plan_Update(0, 0, 1, SPEED_STRAIGHT_TARGET);
        Speed_Target_Tick();
    }
    assert(fabsf(speed_target-SPEED_STRAIGHT_TARGET) < 0.01f);
    for (int i=0; i<40; ++i)
    {
        Speed_Plan_Update(30, 0, 1, SPEED_STRAIGHT_TARGET);
        Speed_Target_Tick();
    }
    assert(fabsf(speed_target-SPEED_CURVE_TARGET) < 0.01f);
    Speed_Stop_Latch();
    Speed_Target_Tick();
    assert(speed_target == 0);
    for (int i=0; i<250; ++i)
    {
        Speed_Plan_Update(0, 0, 1, SPEED_STRAIGHT_TARGET);
        Speed_Target_Tick();
        assert(speed_target == 0); // Later normal images cannot restart the car.
    }
    puts("PASS: stripes/noise/221 curves, GO90, exit spike rejection, steering rate, speed release gate, timeout, rollover, LCD bounds, start-line ignore, clear-road arming, finish latch, actual speed planner");
    return 0;
}
