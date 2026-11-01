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
enum {PWM2_MODULE0_CHA_C6,PWM2_MODULE1_CHA_C8,PWM4_MODULE2_CHA_C30};
enum {C7,C9,GPIO_LOW};
static uint32_t duty[3];
static int16_t encoder_data_1=28,encoder_data_2=28;
static const float servo_motor_duty_middle=(SERVO_MOTOR_R_MAX+SERVO_MOTOR_L_MAX)/2.f;
static void pwm_set_duty(int c,uint32_t v){assert(v<=PWM_DUTY_MAX);duty[c]=v;}
static void gpio_set_level(int p,int l){(void)p;(void)l;}
static void tft180_show_string(int x,int y,const char*s){(void)x;(void)y;(void)s;}
#include "frame_control.inc"
static void scene(int island) {
    for(int y=0;y<120;++y)for(int x=0;x<188;++x) {
        int open=island && y>20 && y<80;
        int black=island && (x-55)*(x-55)*64+(y-60)*(y-60)*400<=25600;
        image[y][x]=(x>(open?-1:25) && x<163 && !black)?255:0;
    }
}
static void frame(void){control_frame();for(int i=0;i<5;++i)motor_tick();}
int main(void){
    My_Pid_Init();
    for(int i=0;i<20;++i){scene(0);frame();}
    assert(track_nav.phase==NAV_NORMAL && track_nav.learned[96]);
    for(int i=0;i<10;++i){scene(1);frame();}
    assert(track_detector.confirmed==TRACK_ROUND_LEFT);
    assert(track_nav.phase==NAV_APPROACH && track_nav.drive_valid);
    for(int i=0;i<130;++i){scene(1);frame();}
    assert(track_nav.phase==NAV_ENTER && track_nav.drive_valid);
    assert(control_mid[96]<94 && duty[SERVO_MOTOR_PWM]>750);
    float entry_min=servo_motor_duty_middle+(SERVO_MOTOR_L_MAX-servo_motor_duty_middle)*ROUND_V2_ENTRY_STEER_FRACTION;
    assert(duty[SERVO_MOTOR_PWM]>=(uint32_t)SERVO_MOTOR_DUTY(entry_min));
    for(int i=0;i<8;++i){memset(image,0,sizeof image);frame();}
    assert(!track_nav.drive_valid && speed_target==0);
    puts("PASS: real camera extraction -> V2 recognition -> left servo -> loss stop");
}
