#ifndef _PID_H
#define _PID_H

#include "zf_common_headfile.h"

// 左右轮共用动态目标速度，单位：编码器计数/采样周期（不是 RPM）。
#define SPEED_STRAIGHT_TARGET    (100.0f)
#define SPEED_CURVE_TARGET       (75.0f)
#define SPEED_BEND_START         (5.0f)   // 像素偏差不超过此值时使用直道速度
#define SPEED_BEND_FULL          (30.0f)  // 达到此值时使用最低弯道速度
#define SPEED_ACCEL_STEP         (0.40f)  // 每10ms目标速度增加量
#define SPEED_DECEL_STEP         (1.20f)  // 每10ms目标速度减少量
#define SPEED_IMAGE_TIMEOUT_MS   (300)
#define SPEED_SAMPLE_PERIOD_MS   (10)

typedef struct
{
    float                kp;                    //P
    float                ki;                    //I
    float                kd;                    //D
    float                imax;                  //积分限幅

    float                out_p;                 //KP输出
    float                out_i;                 //KI输出
    float                out_d;                 //KD输出
    float                out;                   //pid输出
    float                outmax;                //输出限幅

    float                integrator;            //< 位置式控制的积分输出累积值（PWM单位）
    float                last_error;            //< 上次误差
    float                last_derivative;       //< 上次误差与上上次误差之差
    unsigned long        last_t;                //< 上次时间
}pid_param_t;

extern pid_param_t speed_pid_l;  // 电机PID
extern pid_param_t speed_pid_r;  // 电机PID
extern float speed_KP, speed_KI,speed_KD, speed_IMAX, speed_OUTMAX;
extern volatile float speed_target;             // 当前平滑后的目标速度
extern volatile uint32 control_time_ms;          // 10ms中断单调时钟（允许回绕）
extern volatile uint32 speed_distance_counts;   // 两轮平均有符号位移，按uint32回绕
extern float speed_real;                        // 实际速度
extern float speed_pwm;                         // 施加在电机上的PWM占空比


void My_Pid_Init(void);
void Speed_Plan_Update(float near_error, float line_change, uint8 valid, float speed_limit);
void Speed_Stop_Latch(void); // 终点停车锁定，仅复位/重新上电解除。
void Speed_Odom_Update(short left_count, short right_count); // 每10ms读取编码器后调用
void Speed_Target_Tick(void); // 仅在10ms速度控制中断中调用
void Pid_Param_Init(pid_param_t * pid, float kp, float ki, float kd, float imax, float outmax);

float constrain_float(float amt, float low, float high);
short constrain_short(short amt, short low, short high);


float PidLocCtrl(pid_param_t * pid, float error, float t);
float PidIncCtrl(pid_param_t * pid, float error , float t);


#endif
