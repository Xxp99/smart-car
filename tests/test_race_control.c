// 主循环/中断控制集成测试：检查特殊元素接管顺序、速度停车锁存和显示文本缓冲长度。
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "pid.h"
#include "main.h"
#include "zebra_pass.h"
#include "brick_avoid.h"
#define MT9V03X_H 120
#define MT9V03X_W 188
#define PWM_DUTY_MAX 10000
enum {PWM2_MODULE0_CHA_C6, PWM2_MODULE1_CHA_C8, PWM4_MODULE2_CHA_C30};
enum {C7, C9, GPIO_LOW};
static uint32_t duty[3];
static int16_t encoder_data_1=28, encoder_data_2=28;
static char screen82[22], screen92[22];
static const float servo_motor_duty_middle=(SERVO_MOTOR_R_MAX+SERVO_MOTOR_L_MAX)/2.f;
static void pwm_set_duty(int channel, uint32_t value)
{
    assert(value <= PWM_DUTY_MAX);
    duty[channel]=value;
}
static void gpio_set_level(int pin, int level) { (void)pin; (void)level; }
static void tft180_show_string(int x, int y, const char *s)
{
    assert(x==0 && strlen(s)==21 && strlen(s)*6<=128 && y+7<160);
    strcpy(y==82 ? screen82 : screen92, s);
}
// Generated verbatim from main.c and isr.c, so this tests the production wiring.
#include "frame_control.inc"

static void road(int opening)
{
    memset(image, 0, sizeof(image));
    for (int y=0; y<120; ++y)
    {
        int l=50, r=138;
        if (y>=80 && y<=100)
        {
            if (opening&1) l=0;
            if (opening&2) r=187;
        }
        for (int x=l+1; x<r; ++x) image[y][x]=255;
    }
}
static void stripes(void)
{
    road(0);
    for (int y=60; y<90; ++y)
        for (int x=40; x<148; ++x) image[y][x]=(x/6)%2 ? 255 : 0;
}
static void partial_ring_image(int side)
{
    memset(image,0,sizeof(image));
    for (int y=0; y<120; ++y)
    {
        int l=90, r=178;
        if (y>=80 && y<=100) l=0;
        if (y>=94 && y<=97) r=187;
        if (side==2) { int old_l=l; l=187-r; r=187-old_l; }
        for (int x=l+1; x<r; ++x) image[y][x]=255;
    }
}
static void frame(void)
{
    control_frame();
    for (int i=0; i<5; ++i) motor_tick();
    for (int y=0; y<120; ++y)
        assert(control_left[y]<188 && control_right[y]<188 && control_mid[y]<188);
}
static void clear_frames(int count)
{
    for (int i=0; i<count; ++i) { road(0); frame(); }
}
static void gap_frames(int side, int count)
{
    for (int i=0; i<count; ++i) { road(side); frame(); }
}
static void entry_curve_frames(int side,int count)
{
    for(int i=0;i<count;++i) {
        memset(image,0,sizeof image);
        for(int y=0;y<120;++y) {
            int shift=(y-96)/2;
            if(shift < -12) shift=-12;
            if(side==2) shift=-shift;
            for(int x=51+shift;x<138+shift;++x) image[y][x]=255;
        }
        frame();
    }
}

static void abc_enter(int side) {
    memset(&track_nav,0,sizeof track_nav); memset(&track_detector,0,sizeof track_detector);
    clear_frames(10);
    for(int stage=0;stage<2;++stage) for(int f=0;f<4;++f) {
        for(int y=0;y<120;++y) for(int x=0;x<188;++x) {
            int xx=side==2?x:187-x, border=188;
            if(!stage && y>=85) border=140;
            int by=stage?70:50, span=stage?10:15;
            if(y>=by-span && y<=by+span) border=120+2*abs(y-by);
            int roof=stage && xx>=80 && y<=45-abs(xx-110)/2;
            image[y][x]=xx>45 && xx<border && !roof?255:0;
        }
        frame();
    }
    assert(track_nav.phase==NAV_ENTER && track_nav.drive_valid);
    assert(side==1?control_mid[96]<94:control_mid[96]>94);
    assert(duty[SERVO_MOTOR_PWM]!=750);
}

#ifndef REPLAY_MODE
int main(void)
{
    My_Pid_Init();
#if !ZEBRA_ENABLE
    clear_frames(100);
    for(int i=0;i<200;++i) {stripes(); frame(); assert(zebra.phase==ZEBRA_NORMAL);}
    clear_frames(100);
    assert(zebra_race.phase==ZEBRA_RACE_START);
    assert(fabsf(speed_target-SPEED_STRAIGHT_TARGET)<0.01f);
    assert(strstr(screen92,"Z:OFF"));
    for(int side=1;side<=2;++side) abc_enter(side);
    for(int side=1;side<=2;++side) {
        memset(&brick,0,sizeof brick);
        memset(&track_nav,0,sizeof track_nav);
        memset(&track_detector,0,sizeof track_detector);
        encoder_data_1=encoder_data_2=28;
        clear_frames(10);
        for(int f=0;f<12;++f) {
            road(0);
            for(int y=50;y<=75;++y)
                for(int x=side==1?51:108;x<(side==1?81:138);++x) image[y][x]=0;
            frame();
        }
        assert(brick.phase==BRICK_AVOID && brick.side==side);
        assert(track_nav.phase==NAV_NORMAL && track_nav.width[72]==88);
        assert(side==1?control_mid[96]>94:control_mid[96]<94);
        assert(speed_target>0 && speed_target<=BRICK_SPEED && duty[SERVO_MOTOR_PWM]!=750);
        encoder_data_1=encoder_data_2=0;
        clear_frames(30);
        assert(brick.phase==BRICK_HOLD && brick.travel==0);
        assert(side==1?control_mid[96]>94:control_mid[96]<94);
        encoder_data_1=encoder_data_2=45;
        for(int f=0;f<65 && brick.phase!=BRICK_IDLE;++f) clear_frames(1);
        assert(brick.phase==BRICK_IDLE && control_mid[96]==94);
    }
    puts("PASS: zebra disabled, LCD OFF, mirrored entry steering and straight-pass rejection");
    return 0;
#endif
    clear_frames(10);
    assert(zebra_race.phase==ZEBRA_RACE_START);
    // The start stripe can remain/reappear for longer than the start guard.
    for (int i=0; i<200; ++i)
    {
        stripes(); frame();
        assert(zebra_race.phase==ZEBRA_RACE_START && speed_target>0);
        assert(track_nav.phase==NAV_NORMAL && track_detector.confirmed==TRACK_NONE);
        if (i==2) assert(zebra.phase==ZEBRA_STRAIGHT && duty[SERVO_MOTOR_PWM]==750);
    }
    assert(fabsf(speed_target-ZEBRA_PASS_SPEED)<0.01f); // Independent zebra limit.
    clear_frames(80);
    assert(zebra.phase==ZEBRA_NORMAL && zebra_race.phase==ZEBRA_RACE_ARMED);
    assert(fabsf(speed_target-SPEED_STRAIGHT_TARGET)<0.01f);
    assert(track_nav.learned[96] && track_nav.learned[72]);

    // The main loop really enables cross bridging and caps its speed.
    gap_frames(3, 20);
    assert(track_nav.phase==NAV_CROSS && control_mid[96]==94);
    assert(duty[SERVO_MOTOR_PWM]==750 && fabsf(speed_target-SPEED_CURVE_TARGET)<0.01f);
    assert(strstr(screen82, "CROSS"));
    clear_frames(60);
    assert(track_nav.phase==NAV_NORMAL && fabsf(speed_target-SPEED_STRAIGHT_TARGET)<0.01f);

    // Opening without island B is no longer a ring candidate.
    gap_frames(1,8); assert(track_detector.confirmed!=TRACK_ROUND_LEFT);
    clear_frames(60);
    for(int side=1;side<=2;++side) { abc_enter(side); clear_frames(220); }

    // One noisy candidate must preserve ring state and freeze its evidence.
    abc_enter(1);
    assert(track_nav.phase==NAV_ENTER);
    stripes(); frame();
    assert(track_nav.phase==NAV_ENTER && track_nav.element==TRACK_ROUND_LEFT);
    assert(track_nav.feature_frames==0 && zebra.phase==ZEBRA_NORMAL);
    road(0); frame();
    assert(track_nav.element==TRACK_ROUND_LEFT);
    clear_frames(200); // Let the element timeout recover before the next case.

    // A confirmed crossing with no remaining geometry stops after its short hold.
    gap_frames(3,20);
    for(int i=0;i<8;++i) { memset(image,255,sizeof image); frame(); }
    assert(track_nav.phase==NAV_SLOW && !track_nav.drive_valid && speed_target==0);
    assert(duty[MOTOR1_PWM]==0 && duty[MOTOR2_PWM]==0);
    clear_frames(80);
    assert(speed_target>0 && zebra_race.phase==ZEBRA_RACE_ARMED);

    // A return stripe has priority even if an element was just entered.
    abc_enter(1);
    assert(track_nav.phase==NAV_ENTER);
    for (int i=0; i<2; ++i)
    {
        stripes(); frame();
        assert(zebra_race.phase==ZEBRA_RACE_ARMED && speed_target>0);
        assert(track_nav.phase==NAV_ENTER);
    }
    speed_pid_l.integrator=speed_pid_r.integrator=500;
    stripes(); frame();
    assert(zebra_race.phase==ZEBRA_RACE_PASS && speed_target>0);
    for(int i=0;i<40;++i) {
        stripes(); frame();
        assert(zebra_race.phase==ZEBRA_RACE_PASS && speed_target>0);
    }
    // Crossing confirmed but the wheels are stationary: no finish by elapsed time.
    encoder_data_1=encoder_data_2=0;
    clear_frames(40);
    assert(zebra_race.phase==ZEBRA_RACE_CLEAR && speed_target>0);
    assert(speed_target<=ZEBRA_PASS_SPEED+0.01f); // No straight-road acceleration before stop.
    // Real ISR accumulation reaches clearance, then main latches and ISR zeros outputs.
    encoder_data_1=encoder_data_2=35;
    for(int i=0;i<40 && zebra_race.phase!=ZEBRA_RACE_STOP;++i) { road(0); frame(); }
    assert(zebra_race.phase==ZEBRA_RACE_STOP && speed_target==0);
    assert(duty[MOTOR1_PWM]==0 && duty[MOTOR2_PWM]==0 && speed_pwm==0);
    assert(speed_pid_l.integrator==0 && speed_pid_r.integrator==0);
    assert(speed_pid_l.out==0 && speed_pid_r.out==0);
    assert(duty[SERVO_MOTOR_PWM]==750 && strstr(screen82, "STOP") && strstr(screen92, "Z:STOP"));
    clear_frames(200);
    assert(speed_target==0 && duty[MOTOR1_PWM]==0 && duty[MOTOR2_PWM]==0);
    assert(duty[SERVO_MOTOR_PWM]==750 && zebra_race.phase==ZEBRA_RACE_STOP);
    puts("PASS: actual main/ISR integration, partial clipping image-to-control, early candidate slowdown, departure GO90, cross/left/right navigation, zebra priority, finish PWM/integrator zero, persistent stop, LCD bounds");
    return 0;
}

#else
// Packed records: uint32 little-endian milliseconds + 188*120 grayscale bytes.
// Motor feedback is mocked: this validates vision/control decisions, not vehicle dynamics.
int main(int argc, char **argv)
{
    if(argc!=3) return 2;
    FILE *input=fopen(argv[1],"rb"), *output=fopen(argv[2],"w");
    if(!input || !output) return 3;
    My_Pid_Init();
    fprintf(output,"ms,candidate,phase,zebra,guide,drive,near_mid,far_mid,target,servo_pwm\n");
    uint32_t timestamp;
    while(fread(&timestamp,sizeof timestamp,1,input)==1)
    {
        if(fread(image,1,sizeof image,input)!=sizeof image) return 4;
        if(timestamp<control_time_ms || timestamp%SPEED_SAMPLE_PERIOD_MS) return 5;
        while(control_time_ms<timestamp) motor_tick();
        get_hist_gram(&image[0][0],120,188,hist_gram);
        unsigned char threshold=get_threshold_otsu(hist_gram);
        binaryzation_process(&image[0][0],120,188,threshold);
        control_frame();
        fprintf(output,"%u,%s,%s,%s,%u,%u,%u,%u,%.2f,%u\n",
            timestamp,track_element_name(track_detector.candidate),track_nav_name(track_nav.phase),
            zebra_status(&zebra,&zebra_race,zebra_detect(&image[0][0],120,188)),
            track_nav.guide_valid,track_nav.drive_valid,control_mid[96],control_mid[72],
            speed_target,duty[SERVO_MOTOR_PWM]);
    }
    fclose(input); fclose(output);
    return 0;
}
#endif
