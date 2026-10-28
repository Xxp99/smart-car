#include "main.h" // 所有引脚信息更改在main.h里改宏

// *************************** 例程硬件连接说明 ***************************
/*

            连接好电机、主板、驱动板、摄像头、编码器
            (请确保全部正确连接后在使用此例程，所有外设均会使用)
*/
// *************************** 例程测试说明 ***************************
/*
            本例程包含功能：
            1.初始化摄像头并采集图像进行大津法二值化
            2.初始化屏幕并显示关键信息
            3.根据偏差计算最基本的舵机打角
            4.运用最基本pid参数和pid算法使电机旋转 (pid参数在pid.c中)


            注意！注意！注意！注意！
            注意！注意！注意！注意！
            注意！注意！注意！注意！

            1.请仔细阅读main.h的所有注释，确保所有宏都和自己的硬件连接对应
            2.若屏幕显示二值化后的图像有缺陷，可能不能正常走直线，
            可以自行更改二值化算法或找光环境良好且均匀的环境测试例程（例程不提供更多的二值化算法）

            无线串口看图（VOFA+）：
            1.无线串口模块接 UART8：模块 TX->D17、模块 RX->D16、模块 RTS->D26
            2.VOFA+ 数据引擎选 FireWater，串口选无线模块对应的 COM 口，波特率与
              zf_device_wireless_uart.h 里的 WIRELESS_UART_BUAD_RATE 一致（默认 115200）
            3.拖一个「图像」控件到画布，右键 -> 指定图片 -> img0，就能看到摄像头图像
            4.本例程只把图像"顺便"发给上位机：发送过程被切成很多小段，
              每轮主循环只写一小段，所以发图时舵机/电机控制照常运行。
              发图速度和是否发图，都在 img_uart.h 顶部配置，不用改 main.c。

*/

// **************************** 代码区域 ****************************

// 舵机动作角度中值
const float servo_motor_duty_middle = (SERVO_MOTOR_R_MAX + SERVO_MOTOR_L_MAX) / 2.f; 

void Init()
{

    // 初始化flash, 储存参数. 一个扇区有8页, 一页可以储存4096字节, 一个参数占4个字节, 因此一页最多只能存64个参数
    flash_init();

    // 本界面按128x160竖屏布局；初始化提示也使用6x8字体以免越界。
    tft180_set_dir(TFT180_PORTAIT);
    tft180_init();
    tft180_set_font(TFT180_6X8_FONT);

    // mt9v03x摄像头初始化
    while (1)
    {
        if (mt9v03x_init())
            tft180_show_string(0, 16, "mt9v03x reinit.");
        else
            break;
        system_delay_ms(1000);
    }
    tft180_show_string(0, 16, "init success.");
    tft180_clear();

    // 编码器初始化
    encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B); // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B); // 初始化编码器模块与引脚 正交解码编码器模式

    // 电机初始化
    pwm_init(MOTOR1_PWM, 17000, 0); // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(MOTOR1_DIR, GPO, MOTOR1_FORWARD_DIR_LEVEL, GPO_PUSH_PULL); // 初始化为已配置的前进方向
    pwm_init(MOTOR2_PWM, 17000, 0); // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(MOTOR2_DIR, GPO, MOTOR2_FORWARD_DIR_LEVEL, GPO_PUSH_PULL); // 初始化为已配置的前进方向

    // 舵机
    pwm_init(SERVO_MOTOR_PWM, SERVO_MOTOR_FREQ, 0);

    // PID初始化
    My_Pid_Init();

    // 固定速度闭环的采样周期，与目标速度的计数单位对应。
    pit_ms_init(PIT_CH0, SPEED_SAMPLE_PERIOD_MS);
}

uint8_t image[MT9V03X_H][MT9V03X_W];
short hist_gram[256];
uint8_t left_line[MT9V03X_H];
uint8_t mid_line[MT9V03X_H];
uint8_t right_line[MT9V03X_H];
track_detector_t track_detector = {TRACK_NONE, TRACK_NONE, 0, 0};
track_nav_t track_nav = {0};
uint8_t control_left[MT9V03X_H], control_right[MT9V03X_H], control_mid[MT9V03X_H];

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M); // 不可删除
    debug_init();                  // 调试端口初始化
    system_delay_ms(300);          // 等待主板其他外设上电完成

    // 0 表示无线模块(V2.0 以上)自动波特率成功；非 0 时图像发不出去，见 img_uart.h 里的说明
    uint8 wireless_init_state = wireless_uart_init();
    Init();
    img_uart_init(wireless_init_state); // 图像发送状态机复位，必须在 Init() 之后
    interrupt_global_enable(0);

    while (1)
    {
        //  mt9v03x摄像头
        if (mt9v03x_finish_flag)
        {
            // 另寻空间将图像保存下来，以免产生因读写冲突带来的未知后果
            memcpy((uint8_t *)image, (uint8_t *)mt9v03x_image, sizeof(uint8_t) * MT9V03X_H * MT9V03X_W);
            // 要发原始灰度图时，在这里拷一份给上位机（此刻 image 里还是摄像头灰度，还没二值化）
            IMG_UART_CAPTURE_BEFORE_BINARIZE((const uint8 *)image);
            // 获取直方图
            get_hist_gram((uint8 *)image, MT9V03X_H, MT9V03X_W, hist_gram);
            unsigned char threshold = get_threshold_otsu(hist_gram);
            // 二值化处理
            binaryzation_process((uint8 *)image, MT9V03X_H, MT9V03X_W, threshold);
            // 边界线寻找
            auxiliary_process((uint8 *)image, MT9V03X_H, MT9V03X_W, threshold, left_line, mid_line, right_line);
            track_element_update(&track_detector,
                track_element_classify(left_line, right_line, MT9V03X_H, MT9V03X_W));
            track_nav_update(&track_nav, &track_detector, left_line, right_line,
                control_left, control_right, control_mid, MT9V03X_H, MT9V03X_W, control_time_ms);

            // 转向仍看4/5行；额外用3/5行的中线趋势提前安排减速。
            uint8_t near_row = MT9V03X_H * 4 / 5;
            uint8_t far_row = MT9V03X_H * 3 / 5;
            float mid_err = (MT9V03X_W / 2) - control_mid[near_row];
            uint8_t speed_line_valid =
                left_line[near_row] > 0 && right_line[near_row] < MT9V03X_W - 1 &&
                left_line[near_row] < right_line[near_row] &&
                left_line[far_row] > 0 && right_line[far_row] < MT9V03X_W - 1 &&
                left_line[far_row] < right_line[far_row];
            // 导航识别失败只降速，不再进入锁定停车状态。
            float speed_limit = track_nav.phase == NAV_NORMAL ? SPEED_STRAIGHT_TARGET : SPEED_CURVE_TARGET;
            Speed_Plan_Update(mid_err, (float)control_mid[far_row] - control_mid[near_row],
                              speed_line_valid, speed_limit);

            // 控制先于阻塞式刷屏，避免整帧显示耗时延迟舵机更新。
            float tmp_duty = servo_motor_duty_middle - mid_err * 0.5 * SERVO_DIR;
            tmp_duty = MAX(tmp_duty,MIN(SERVO_MOTOR_L_MAX,SERVO_MOTOR_R_MAX));
            tmp_duty = MIN(tmp_duty,MAX(SERVO_MOTOR_L_MAX,SERVO_MOTOR_R_MAX));
            pwm_set_duty(SERVO_MOTOR_PWM, SERVO_MOTOR_DUTY(tmp_duty));

            for (uint8_t _i = 0; _i < MT9V03X_H; ++_i)
            {
                // 将边界线也显示出来
                image[_i][left_line[_i]] = 0;
                image[_i][control_mid[_i]] = 0;
                image[_i][control_left[_i]] = 0;
                image[_i][control_right[_i]] = 0;
                image[_i][right_line[_i]] = 0;
            }
            // 要发二值化+补线图时，在这里拷一份给上位机（IMG_UART_SOURCE 选 BINARY 才生效）
            IMG_UART_CAPTURE_AFTER_BINARIZE((const uint8 *)image);
            // 显示图像
            tft180_displayimage03x((uint8_t *)image, 125, 90);
            // E=当前元素，S=行驶阶段；图像叠加的是控制用补线。
            tft180_show_string(0, 92, "E:");
            tft180_show_string(12, 92, track_element_name(track_nav.element));
            tft180_show_string(54, 92, "S:");
            tft180_show_string(66, 92, track_nav_name(track_nav.phase));

            // 图像占用 y=0..89；识别状态及数字在下方，6x8 字体。
            // 保留完整 int16 位数，避免三位显示截断真实异常值。
            tft180_show_string(0, 102, "L:");
            tft180_show_int(36, 102, encoder_data_1, 5);
            tft180_show_string(0, 112, "R:");
            tft180_show_int(36, 112, encoder_data_2, 5);
            tft180_show_string(0, 122, "LMAX:");
            tft180_show_int(36, 122, encoder_left_max, 5);
            tft180_show_string(0, 132, "LBAD:");
            tft180_show_uint(36, 132, encoder_left_abnormal_count, 10);
            tft180_show_string(0, 142, "PWM2:");
            tft180_show_float(36, 142, speed_pwm, 4, 2);
            tft180_show_string(0, 152, "TGT:");
            tft180_show_float(36, 152, speed_target, 2, 1);

            // 处理完一帧图像后务必把该标志位清零！
            mt9v03x_finish_flag = 0;
        }

        // 无线串口图像发送：每轮只往串口写一小段就返回，所以发图时控制照常运行
        img_uart_task();
    }
}
