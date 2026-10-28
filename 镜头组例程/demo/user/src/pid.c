#include "pid.h"

// 电机PI初始参数：保持原有每10ms调用、t=1的离散周期约定。
// Ki=0.5 是待实车验证的起始值，不能直接按秒制PID参数使用。
float speed_KP = 50, speed_KI = 0.5f,speed_KD = 0.0, speed_IMAX = 5000.0, speed_OUTMAX = 5000.0;
volatile float speed_target = SPEED_CURVE_TARGET;
volatile uint32 control_time_ms = 0;
volatile uint32 speed_distance_counts = 0;
static volatile float speed_requested = SPEED_CURVE_TARGET;
static volatile uint16 speed_image_age = SPEED_IMAGE_TIMEOUT_MS / SPEED_SAMPLE_PERIOD_MS;
static volatile uint8 speed_stop_latched = 0;
float speed_real = 0.0;
float speed_pwm = 0.0;

pid_param_t speed_pid_l;  // 电机PID
pid_param_t speed_pid_r;  // 电机PID

// 主循环每帧提交一次速度要求。近处偏差和远近中线变化都可触发减速。
void Speed_Plan_Update(float near_error, float line_change, uint8 valid, float speed_limit)
{
    if (speed_stop_latched) return;
    float bend = MAX(func_abs(near_error), func_abs(line_change));
    float ratio = constrain_float((bend - SPEED_BEND_START) /
                                  (SPEED_BEND_FULL - SPEED_BEND_START), 0.0f, 1.0f);
    float planned = valid ? SPEED_STRAIGHT_TARGET -
        ratio * (SPEED_STRAIGHT_TARGET - SPEED_CURVE_TARGET) : SPEED_CURVE_TARGET;
    speed_requested = MIN(planned, constrain_float(speed_limit, 0.0f, SPEED_STRAIGHT_TARGET));
    speed_image_age = 0;
}

// 用真实反馈累计位移，不以目标速度或经过时间替代车辆移动。
void Speed_Odom_Update(short left_count, short right_count)
{
    int delta = ((int)left_count + (int)right_count) / 2;
    speed_distance_counts += (uint32)delta;
}

void Speed_Stop_Latch(void)
{
    // 中断先检查锁存位；后续图像即使申请加速也不能再次启动。
    speed_stop_latched = 1;
}

// 目标速度的变化率由固定采样周期决定，不随摄像头帧率变化。
void Speed_Target_Tick(void)
{
    float requested;
    control_time_ms += SPEED_SAMPLE_PERIOD_MS;
    if (speed_image_age < SPEED_IMAGE_TIMEOUT_MS / SPEED_SAMPLE_PERIOD_MS)
    {
        ++speed_image_age;
    }
    requested = speed_stop_latched || speed_image_age >= SPEED_IMAGE_TIMEOUT_MS / SPEED_SAMPLE_PERIOD_MS ?
        0.0f : speed_requested;
    if (requested <= 0.0f)
    {
        speed_target = 0.0f;
        return;
    }
    speed_target += constrain_float(requested - speed_target,
                                    -SPEED_DECEL_STEP, SPEED_ACCEL_STEP);
    speed_target = constrain_float(speed_target, 0.0f, SPEED_STRAIGHT_TARGET);
}

/*******************************************************************************
* 函 数 名         : My_Pid_Init
* 函数功能         : PID初始化	
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void My_Pid_Init(void)
{
    Pid_Param_Init(&speed_pid_l, speed_KP, speed_KI,speed_KD, speed_IMAX, speed_OUTMAX);
    Pid_Param_Init(&speed_pid_r, speed_KP, speed_KI,speed_KD, speed_IMAX, speed_OUTMAX);
}


/*******************************************************************************
* 函 数 名         : Pid_Param_Init
* 函数功能         : PID参数初始化	
* 输    入         : imax:积分项最大值
* 输    出         : 无
*******************************************************************************/
void Pid_Param_Init(pid_param_t * pid, float kp, float ki, float kd, float imax, float outmax)
{
    pid->kp        = kp;
    pid->ki        = ki;
    pid->kd        = kd;
    pid->imax      = imax;
    pid->out_p     = 0;
    pid->out_i     = 0;
    pid->out_d     = 0;
    pid->out       = 0;
    pid->outmax    = outmax;
    pid->integrator= 0;
    pid->last_error= 0;
    pid->last_derivative   = 0;
    pid->last_t    = 0;
}

/*******************************************************************************
* 函 数 名         : PidLocCtrl
* 函数功能	   : 位置式PID控制
* 输    入         : pid, error, t
* 输    出         : float
*******************************************************************************/
float PidLocCtrl(pid_param_t * pid, float error, float t)
{
    // 当前调用仅用于单向电机速度环，实际PWM范围为0..outmax。
    float next_i;
    float next_out;
    pid->out_p = pid->kp * error;
    pid->out_d = pid->kd/t * (error - pid->last_error);
    // 保存并限制积分输出本身，避免原代码的限幅随后被覆盖。
    next_i = constrain_float(pid->integrator + pid->ki * t * error,
                             -pid->imax, pid->imax);
    next_out = pid->out_p + next_i + pid->out_d;
    // 输出饱和且误差继续推动饱和时暂停积分；反向误差允许积分恢复。
    if (!((next_out > pid->outmax && error > 0.0f) ||
          (next_out < 0.0f && error < 0.0f)))
    {
        pid->integrator = next_i;
    }
    pid->out_i = pid->integrator;
    pid->last_error = error;
    pid->out = pid->out_p + pid->out_i + pid->out_d;
    pid->out = constrain_float(pid->out, 0.0f, pid->outmax);

    return pid->out;
}


/*******************************************************************************
* 函 数 名         : PidIncCtrl
* 函数功能	   : 增量式PID控制
* 输    入         : pid, error, t
* 输    出         : float
*******************************************************************************/
float PidIncCtrl(pid_param_t * pid, float error, float t)
{

    pid->out_p = pid->kp * (error - pid->last_error);
    pid->out_i = pid->ki * error * t ;
    pid->out_d = pid->kd/t * ((error - pid->last_error) - pid->last_derivative);

    pid->last_derivative = error - pid->last_error;
    pid->last_error = error;

    pid->out += pid->out_p + pid->out_i + pid->out_d;

    pid->out = constrain_float(pid->out, -pid->outmax, pid->outmax);
    return pid->out;
}

/*******************************************************************************
* 函 数 名         : constrain_float
* 函数功能         : 浮点型数限幅
* 输    入         : amt,low,high
* 输    出         : float
*******************************************************************************/
float constrain_float(float amt, float low, float high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

/*******************************************************************************
* 函 数 名         : constrain_short
* 函数功能         : 短整型数限幅
* 输    入         : amt,low,high
* 输    出         : short
*******************************************************************************/
short constrain_short(short amt, short low, short high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

