#include "main.h" // 所有引脚信息更改在main.h里改宏
#include "fsl_clock.h"
#include <stdio.h>
#include "zebra_pass.h"
#include "zebra_lane.h"
#include "brick_avoid.h"
#if WIRELESS_SPEED_ENABLE
#include "wireless_speed.h"
#endif
#if WIRELESS_IMAGE_ENABLE
#include "wireless_image.h"
#endif

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

*/

// **************************** 代码区域 ****************************

// 舵机动作角度中值
const float servo_motor_duty_middle = (SERVO_MOTOR_R_MAX + SERVO_MOTOR_L_MAX) / 2.f; 

#if C30_LEVEL_TEST_ENABLE
// C30电平检查：只验证目标引脚是否输出高低电平，不应连接舵机或电机负载。
static void C30_Level_Test_Run(void)
{
    // 不启动摄像头、编码器、速度PID及PIT；驱动电机始终零占空比。
    pwm_init(MOTOR1_PWM, 17000, 0);
    pwm_init(MOTOR2_PWM, 17000, 0);
    gpio_init(MOTOR1_DIR, GPO, MOTOR1_FORWARD_DIR_LEVEL, GPO_PUSH_PULL);
    gpio_init(MOTOR2_DIR, GPO, MOTOR2_FORWARD_DIR_LEVEL, GPO_PUSH_PULL);
    // 本模式不初始化舵机PWM，C30直接作为推挽GPIO输出。
    gpio_init(C30, GPO, GPIO_LOW, GPO_PUSH_PULL);

    tft180_set_dir(TFT180_PORTAIT);
    tft180_init();
    tft180_set_font(TFT180_6X8_FONT);
    tft180_clear();
    tft180_show_string(0, 0, "C30 LEVEL TEST");
    tft180_show_string(0, 16, "UNPLUG SERVO!");
    tft180_show_string(0, 32, "MOTORS: OFF");
    tft180_show_string(0, 64, "INTERVAL: 3 SEC");

    while (1)
    {
        pwm_set_duty(MOTOR1_PWM, 0);
        pwm_set_duty(MOTOR2_PWM, 0);
        gpio_set_level(C30, GPIO_LOW);
        tft180_show_string(0, 48, "C30: LOW ");
        system_delay_ms(3000);
        gpio_set_level(C30, GPIO_HIGH);
        tft180_show_string(0, 48, "C30: HIGH");
        system_delay_ms(3000);
    }
}
#endif

#if SERVO_TEST_ENABLE
// 仅适用于当前C30 / PWM4 SM2 A的边沿对齐测试。显示寄存器推算值，非实测波形。
// 舵机脉宽诊断：逐步输出测试占空比，用于确认PWM频率、舵机中位和左右方向。
static void Servo_PWM_Diagnostics(void)
{
    char row[22];
    uint32 ipg = CLOCK_GetFreq(kCLOCK_IpgClk);
    uint16 ctrl = PWM4->SM[2].CTRL;
    uint16 ctrl2 = PWM4->SM[2].CTRL2;
    uint32 divider = 1u << ((ctrl & PWM_CTRL_PRSC_MASK) >> PWM_CTRL_PRSC_SHIFT);
    uint16 init = PWM4->SM[2].INIT;
    uint16 val1 = PWM4->SM[2].VAL1;
    uint16 val2 = PWM4->SM[2].VAL2;
    uint16 val3 = PWM4->SM[2].VAL3;
    uint16 count_before = PWM4->SM[2].CNT;
    system_delay_ms(1);
    uint16 count_after = PWM4->SM[2].CNT;
    uint32 mux = IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_B1_14] & 0xFu;
    uint8 valid = ((ctrl2 & PWM_CTRL2_CLK_SEL_MASK) == 0 && init == 0 &&
                   val3 >= val2 && val3 <= (uint32)val1 + 1u && ipg != 0);
    if (valid)
    {
        double tick_hz = (double)ipg / divider;
        snprintf(row, sizeof(row), "HZ:%8.2f", tick_hz / ((uint32)val1 + 1u));
        tft180_show_string(0, 30, row);
        snprintf(row, sizeof(row), "US:%8.1f", (val3 - val2) * 1000000.0 / tick_hz);
        tft180_show_string(0, 40, row);
    }
    else
    {
        tft180_show_string(0, 30, "HZ: N/A    ");
        tft180_show_string(0, 40, "US: N/A    ");
    }
    snprintf(row, sizeof(row), "IPG:%9lu", (unsigned long)ipg);
    tft180_show_string(0, 50, row);
    snprintf(row, sizeof(row), "DIV:%3lu CLK:%u", (unsigned long)divider,
             (unsigned)(ctrl2 & PWM_CTRL2_CLK_SEL_MASK));
    tft180_show_string(0, 60, row);
    snprintf(row, sizeof(row), "V1:%5u V2:%5u", (unsigned)val1, (unsigned)val2);
    tft180_show_string(0, 70, row);
    snprintf(row, sizeof(row), "V3:%5u IN:%5u", (unsigned)val3, (unsigned)init);
    tft180_show_string(0, 80, row);
    snprintf(row, sizeof(row), "RUN:%u OUT:%u",
             (unsigned)((PWM4->MCTRL >> (PWM_MCTRL_RUN_SHIFT + 2)) & 1u),
             (unsigned)((PWM4->OUTEN >> (PWM_OUTEN_PWMA_EN_SHIFT + 2)) & 1u));
    tft180_show_string(0, 90, row);
    snprintf(row, sizeof(row), "MASK:%u MUX:%lu",
             (unsigned)((PWM4->MASK >> (PWM_MASK_MASKA_SHIFT + 2)) & 1u), (unsigned long)mux);
    tft180_show_string(0, 100, row);
    snprintf(row, sizeof(row), "CNT:%5u>%5u", (unsigned)count_before, (unsigned)count_after);
    tft180_show_string(0, 110, row);
    tft180_show_string(0, 120, "REG CALC; NOT SCOPE");
}

// 舵机测试模式：独占主循环并按设定角度摆动；测试结束需关闭宏才能恢复循迹。
static void Servo_Test_Run(void)
{
    // 独立启动路径：不初始化摄像头、速度 PID 或 PIT 控制中断。
    pwm_init(MOTOR1_PWM, 17000, 0);
    pwm_init(MOTOR2_PWM, 17000, 0);
    gpio_init(MOTOR1_DIR, GPO, MOTOR1_FORWARD_DIR_LEVEL, GPO_PUSH_PULL);
    gpio_init(MOTOR2_DIR, GPO, MOTOR2_FORWARD_DIR_LEVEL, GPO_PUSH_PULL);
    pwm_init(SERVO_MOTOR_PWM, SERVO_MOTOR_FREQ, SERVO_MOTOR_DUTY(90));

    tft180_set_dir(TFT180_PORTAIT);
    tft180_init();
    tft180_set_font(TFT180_6X8_FONT);
    tft180_clear();
    tft180_show_string(0, 0, "SERVO TEST");
    tft180_show_string(0, 10, "MOTORS: OFF");
    tft180_show_string(0, 20, "ANGLE:");
    tft180_show_uint(42, 20, 90, 3);
    tft180_show_string(0, 140, "FIXED: HOLD 90");
    tft180_show_string(0, 150, "MECHANICAL ALIGN");
    // 上电后始终保持90，供机械中位校准；禁止落入正常循迹流程。
    system_delay_ms(25);
    while (1)
    {
        pwm_set_duty(MOTOR1_PWM, 0);
        pwm_set_duty(MOTOR2_PWM, 0);
        Servo_PWM_Diagnostics();
        system_delay_ms(500);
    }
}
#endif

#if WIRELESS_TEST_ENABLE
static uint8 wireless_test_ready;
static uint32 wireless_test_last_ms;
static uint16 wireless_test_sent, wireless_test_busy;

// 每秒尝试一次；模块忙或未连接时直接跳过，不等待RTS超时。
// 无线文字测试的非阻塞发送轮询，受RTS流控且不与图像/速度遥测共用UART。
static void Wireless_Test_Poll(void)
{
    static const uint8 message[] = "SmartCar5 UART OK\r\n";
    uint32 now = control_time_ms;
    if (!wireless_test_ready || (uint32)(now - wireless_test_last_ms) < 1000u)
        return;
    wireless_test_last_ms = now;
    if (gpio_get_level(WIRELESS_UART_RTS_PIN))
    {
        if (wireless_test_busy < 65535u) ++wireless_test_busy;
        return;
    }
    // 单包小于30字节，按驱动约定检查RTS后发送；115200下约1.65ms。
    // 不调用带100ms忙等待的wireless_uart_send_string，避免未接模块时拖慢图像处理。
    uart_write_buffer(WIRELESS_UART_INDEX, message, sizeof(message) - 1u);
    if (wireless_test_sent < 65535u) ++wireless_test_sent;
}

// 将无线测试发送状态显示到屏幕，便于区分模块未就绪、被流控和已发送。
static void Wireless_Test_Show(void)
{
    char row[22];
    // 21字符，替代LMAX一行；发送计数仅表示交给本机UART，不代表无线接收确认。
    snprintf(row, sizeof(row), "W%u R%u T%05u B%05u  ",
        (unsigned)wireless_test_ready,
        (unsigned)(gpio_get_level(WIRELESS_UART_RTS_PIN) != 0),
        (unsigned)wireless_test_sent, (unsigned)wireless_test_busy);
    tft180_show_string(0, 122, row);
}

#endif

// 初始化摄像头、舵机、电机、编码器、无线串口和固定周期速度控制中断。
void Init()
{

    // 本界面按128x160竖屏布局；初始化提示也使用6x8字体以免越界。
    tft180_set_dir(TFT180_PORTAIT);
    tft180_init();
    tft180_set_font(TFT180_6X8_FONT);
    tft180_clear();
    tft180_show_string(0, 0, "BOOT 1: DISPLAY OK");
    tft180_show_string(0, 16, "BOOT 2: FLASH");
    flash_init();
    tft180_show_string(0, 16, "BOOT 3: CAMERA");

    // mt9v03x摄像头初始化
    while (1)
    {
        if (mt9v03x_init())
            tft180_show_string(0, 16, "mt9v03x reinit.");
        else
            break;
        system_delay_ms(1000);
    }
    tft180_show_string(0, 16, "BOOT 4: ENCODER     ");

    // 编码器初始化
    encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B); // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B); // 初始化编码器模块与引脚 正交解码编码器模式

    tft180_show_string(0, 16, "BOOT 5: MOTOR      ");
    // 电机初始化
    pwm_init(MOTOR1_PWM, 17000, 0); // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(MOTOR1_DIR, GPO, MOTOR1_FORWARD_DIR_LEVEL, GPO_PUSH_PULL); // 初始化为已配置的前进方向
    pwm_init(MOTOR2_PWM, 17000, 0); // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(MOTOR2_DIR, GPO, MOTOR2_FORWARD_DIR_LEVEL, GPO_PUSH_PULL); // 初始化为已配置的前进方向

    // 舵机
    pwm_init(SERVO_MOTOR_PWM, SERVO_MOTOR_FREQ, 0);

    // PID初始化
    My_Pid_Init();

#if WIRELESS_SPEED_ENABLE
    tft180_show_string(0, 16, "BOOT 6: SPEED UART ");
    wireless_speed_ready = (wireless_uart_init() == 0);
    NVIC_SetPriority(LPUART8_IRQn, 15);
#elif WIRELESS_IMAGE_ENABLE
    tft180_show_string(0, 16, "BOOT 6: IMAGE UART ");
    wireless_image_ready = (wireless_uart_init() == 0);
    // 图传中断低于默认优先级的PIT/摄像头中断。
    NVIC_SetPriority(LPUART8_IRQn, 15);
#elif WIRELESS_TEST_ENABLE
    tft180_show_string(0, 16, "BOOT 6: WIRELESS   ");
    // UART8: 主板TX=D16、RX=D17，模块RTS=D26，当前波特率115200。
    wireless_test_ready = (wireless_uart_init() == 0);
    wireless_test_last_ms = control_time_ms;
#else
    tft180_show_string(0, 32, "WIRELESS TEST OFF");
#endif
    tft180_show_string(0, 16, "BOOT 7: TIMER      ");

    // 固定速度闭环的采样周期，与目标速度的计数单位对应。
    pit_ms_init(PIT_CH0, SPEED_SAMPLE_PERIOD_MS);
}

uint8_t image[MT9V03X_H][MT9V03X_W];
short hist_gram[256];
uint8_t left_line[MT9V03X_H];
uint8_t mid_line[MT9V03X_H];
uint8_t right_line[MT9V03X_H];
static zebra_pass_t zebra = {0};
static zebra_race_t zebra_race = {0};
static zebra_lane_t zebra_lane = {0};
static float last_servo_angle = ZEBRA_STRAIGHT_ANGLE;
static track_detector_t track_detector = {0};
static track_nav_t track_nav = {0};
// 环岛边缘出口提示需连续命中多帧；单帧贴边不应改变环岛阶段。
static uint8_t ring_edge_turn_frames;
static uint8 appr_seen, enter_seen;
static brick_t brick = {0};
static brick_steering_t brick_servo = {0};
static uint32 last_navigation_frame_ms;
static uint8 navigation_frame_initialized;
static uint8_t stripe_row[MT9V03X_H];
uint8_t control_left[MT9V03X_H], control_right[MT9V03X_H], control_mid[MT9V03X_H];

// 主循环每处理一帧图像：二值化与边线提取→特殊元素调度→速度/舵机控制→显示与遥测。
int main(void)
{
    clock_init(SYSTEM_CLOCK_600M); // 不可删除
    debug_init();                  // 调试端口初始化
    system_delay_ms(300);          // 等待主板其他外设上电完成

#if C30_LEVEL_TEST_ENABLE
    C30_Level_Test_Run();
#elif SERVO_TEST_ENABLE
    Servo_Test_Run();
#endif
    Init();
    interrupt_global_enable(0);
    tft180_show_string(0, 16, "BOOT 8: WAIT CAMERA");

    while (1)
    {
        //  mt9v03x摄像头
        if (mt9v03x_finish_flag)
        {
            // 另寻空间将图像保存下来，以免产生因读写冲突带来的未知后果
            memcpy((uint8_t *)image, (uint8_t *)mt9v03x_image, sizeof(uint8_t) * MT9V03X_H * MT9V03X_W);
#if WIRELESS_IMAGE_ENABLE
            Wireless_Image_Begin((const uint8 *)image);
#endif
            // 获取直方图
            get_hist_gram((uint8_t *)image, MT9V03X_H, MT9V03X_W, hist_gram);
            unsigned char threshold = get_threshold_otsu(hist_gram);
            // 二值化处理
            binaryzation_process((uint8_t *)image, MT9V03X_H, MT9V03X_W, threshold);
            // 边界线寻找
            auxiliary_process((uint8_t *)image, MT9V03X_H, MT9V03X_W, threshold, left_line, mid_line, right_line);
            // 识别只使用原始图像/边线，随后再生成控制用补线。
            memcpy(control_left, left_line, sizeof(control_left));
            memcpy(control_right, right_line, sizeof(control_right));
            memcpy(control_mid, mid_line, sizeof(control_mid));
#if ZEBRA_ENABLE
            // 先检测斑马线，再尝试重建可用中线。原始图像命中但补线无效时，不能接管方向。
            uint8_t zebra_raw_candidate = zebra_detect_lane(&image[0][0], MT9V03X_H, MT9V03X_W, left_line, right_line);
            if (zebra_raw_candidate || zebra.phase != ZEBRA_NORMAL ||
                zebra_race.phase == ZEBRA_RACE_PASS || zebra_race.phase == ZEBRA_RACE_CLEAR)
                zebra_lane_build(&zebra_lane, &image[0][0], track_nav.width,
                    track_nav.learned, control_mid, track_nav.path_valid,
                    MT9V03X_H, MT9V03X_W, control_time_ms);
            else zebra_lane.current = 0;
            uint8_t zebra_candidate = zebra_raw_candidate && zebra_lane.current;
            for (unsigned y = 0; y < MT9V03X_H; ++y)
            {
                unsigned black = 0;
                for (unsigned x = MT9V03X_W / 3; x < MT9V03X_W * 2 / 3; ++x)
                    if (image[y][x] == 0) ++black;
                stripe_row[y] = zebra_transitions(&image[y][MT9V03X_W / 6],
                    MT9V03X_W * 2 / 3, 1) >= 6 || black > MT9V03X_W / 4;
            }

#else
            uint8_t zebra_raw_candidate = 0, zebra_candidate = 0;
#endif
            // 转向仍看4/5行；额外用3/5行的中线趋势提前安排减速。
            uint8_t near_row = MT9V03X_H * 4 / 5;
            uint8_t far_row = MT9V03X_H * 3 / 5;
            uint32 now = control_time_ms;
            if (navigation_frame_initialized && now-last_navigation_frame_ms >= SPEED_IMAGE_TIMEOUT_MS)
            {
                memset(&track_detector, 0, sizeof(track_detector));
                track_nav.feature_frames = track_nav.straight_frames = 0;
            }
            navigation_frame_initialized = 1;
            last_navigation_frame_ms = now;
#if ZEBRA_ENABLE
            // 只有重建后的有效候选才累计确认帧；普通边线恢复需同时满足近、远行稳定。
            uint8_t path_recovered =
                zebra_row_valid(left_line, right_line, stripe_row, near_row, MT9V03X_W) &&
                zebra_row_valid(left_line, right_line, stripe_row, far_row, MT9V03X_W);
            zebra_update(&zebra, zebra_candidate, path_recovered,
                         mid_line[near_row], mid_line[far_row], now);
            zebra_race_update(&zebra_race, &zebra, zebra_candidate, now, speed_distance_counts);
            uint8_t stopped = zebra_race.phase == ZEBRA_RACE_STOP;
            if (stopped) Speed_Stop_Latch(); // 下一次10ms中断清零两轮PWM及PI积分。
            // 连续3帧且车道重建有效后才确认；通过时按重建中线循迹。
            uint8_t finishing = zebra_race.phase == ZEBRA_RACE_PASS ||
                                zebra_race.phase == ZEBRA_RACE_CLEAR;
            uint8_t passing = zebra_slow(&zebra, zebra_candidate) || finishing;
#else
            uint8_t path_recovered = 0, stopped = 0, finishing = 0, passing = 0;
#endif
            track_nav.apex_valid = 0; // No current apex until this frame runs navigation.
            track_nav.entry_guide = 0;
            memset(&track_nav.vision,0,sizeof track_nav.vision);
            if (zebra.phase != ZEBRA_NORMAL || finishing || stopped)
            {
                // 斑马线通过/恢复或终点阶段优先，清除冲突的红砖与元素导航状态。
                memset(&brick,0,sizeof brick);
                // 斑马线确认后才接管；候选仅减速，不销毁现有导航阶段。
                // 保留此前学到的赛道宽度，但清除元素确认和旧路径保持。
                memset(&track_detector, 0, sizeof(track_detector));
                track_nav.apex_hits = 0;
                track_nav.phase = NAV_NORMAL;
                track_nav.element = TRACK_NONE;
                track_nav.fault = NAV_FAULT_NONE;
                memset(&track_nav.vision,0,sizeof track_nav.vision);
                track_nav.feature_frames = track_nav.straight_frames = 0;
                track_nav.last_path_ms = now - NAV_HOLD_MS - 1u;
                memset(track_nav.path_valid, 0, sizeof(track_nav.path_valid));
            }
            else if (zebra_candidate)
            {
                // 斑马线尚在候选确认时只暂停元素计数，保留当前导航阶段和最后可信路径。
                memset(&brick,0,sizeof brick);
                track_nav.apex_hits = 0;
                // 候选图像不能推进元素确认或阶段；暂用此前可信路径。
                track_detector.confirm_count = 0;
                track_detector.ring_history = track_detector.ring_votes = 0;
                track_detector.sequence_side = track_detector.sequence_progress = 0;
                track_nav.feature_frames = track_nav.straight_frames = 0;
                if (track_nav.phase != NAV_NORMAL)
                {
                    memcpy(control_mid, track_nav.last_mid, sizeof(control_mid));
                    track_nav.drive_valid = now-track_nav.last_path_ms <= NAV_HOLD_MS;
                }
            }
            else
            {
                track_element_t raw_kind=track_visual_classify(&track_nav.vision,&image[0][0],left_line,right_line,MT9V03X_H,MT9V03X_W);
                // 红砖只可在普通循迹时新接管；已启动的避让允许完成，避免中途被打断。
                int brick_allowed=brick_owns(&brick) || (track_nav.phase==NAV_NORMAL &&
                    (raw_kind==TRACK_NONE || raw_kind==TRACK_UNKNOWN));
                brick_update(&brick,&image[0][0],left_line,right_line,
                    track_nav.width,track_nav.learned,MT9V03X_H,MT9V03X_W,
                    now,speed_distance_counts,brick_allowed);
                if(brick_owns(&brick)) {
                    // 红砖接管期间冻结元素确认，以避障中线和有效标志替代普通导航路径。
                    // Freeze learned road width and element confirmation during a pass.
                    memset(&track_detector,0,sizeof track_detector);
                    for(int y=0;y<MT9V03X_H;++y)
                        track_nav.path_valid[y]=brick_pair(left_line[y],right_line[y],MT9V03X_W);
                    brick_apply(&brick,&image[0][0],left_line,right_line,
                        track_nav.width,track_nav.learned,control_mid,track_nav.path_valid,
                        MT9V03X_H,MT9V03X_W);
                    track_nav.drive_valid=brick.valid && track_nav.path_valid[near_row] && track_nav.path_valid[far_row];
                } else {
                    // 优先采用当前帧A/B轮廓证据；A/B暂时缺失时，保留旧识别器
                    // 的跨帧开口变化作为后备，日志7/8表明它曾是实际候选来源。
                    if (raw_kind == TRACK_UNKNOWN || raw_kind == TRACK_NONE)
                        track_element_observe(&track_detector,left_line,right_line,
                            MT9V03X_H,MT9V03X_W,now);
                    else
                        abc_element_update(&track_detector,raw_kind);
                    track_nav_update(&track_nav, &track_detector, left_line, right_line, &image[0][0],
                        control_left, control_right, control_mid, MT9V03X_H, MT9V03X_W, now);
                }
            }
            uint8_t ring_inside_mode =
                (track_nav.element == TRACK_ROUND_LEFT || track_nav.element == TRACK_ROUND_RIGHT) &&
                (track_nav.phase == NAV_INSIDE ||
                 (track_nav.phase == NAV_SLOW && track_nav.slow_resume_phase == NAV_INSIDE));
            if (ring_inside_mode && !zebra_candidate && !passing && !stopped &&
                now - track_nav.started_ms >= NAV_RING_EDGE_EXIT_MIN_MS &&
                track_nav.ring_edge_turn_hint)
            {
                // 仅在环内阶段且超过最短行驶时间后累计贴边提示；连续命中才触发出口转向。
                if (ring_edge_turn_frames < NAV_RING_EDGE_EXIT_FRAMES)
                    ++ring_edge_turn_frames;
            }
            else ring_edge_turn_frames = 0;
            if (ring_edge_turn_frames >= NAV_RING_EDGE_EXIT_FRAMES)
            {
                // 交由导航状态机切换出口阶段，不在此处直接改写图像边界。
                track_nav_request_ring_exit(&track_nav, MT9V03X_H, MT9V03X_W, now);
                ring_edge_turn_frames = 0;
            }
            if (passing && !stopped) track_nav.drive_valid = zebra_lane.current;
            if(track_nav.phase==NAV_APPROACH) appr_seen=1;
            if(track_nav.phase==NAV_ENTER) enter_seen=1;
            float mid_err = (MT9V03X_W / 2) - control_mid[near_row];
            float line_change = (float)control_mid[far_row] - control_mid[near_row];
            uint8_t speed_line_valid = passing && zebra_lane.current ? 1 :
                (passing ? path_recovered :
                (track_nav.path_valid[near_row] && track_nav.path_valid[far_row]));
            if (passing && zebra_lane.current)
                mid_err -= 0.25f * line_change;
            // 环岛候选先减速，最近5帧同侧命中3次确认；接管及恢复继续限速。
            uint8_t round_pending = track_detector.candidate == TRACK_ROUND_LEFT ||
                                    track_detector.candidate == TRACK_ROUND_RIGHT;
            uint8_t ring_speed_mode = round_pending ||
                (track_nav.phase != NAV_NORMAL &&
                 (track_nav.element == TRACK_ROUND_LEFT || track_nav.element == TRACK_ROUND_RIGHT));
            float speed_limit = stopped ? 0.0f :
                (ring_speed_mode ? SPEED_RING_TARGET :
                 (passing ? ZEBRA_PASS_SPEED : SPEED_STRAIGHT_TARGET));
            // 环岛补线暂时无效时仍按环岛限速继续运行，不因路径保持超时将速度清零。
            if(brick_owns(&brick) && speed_limit>BRICK_SPEED) speed_limit=BRICK_SPEED;
            Speed_Plan_Update(mid_err, line_change,
                              speed_line_valid, speed_limit);

            // 控制先于阻塞式刷屏，避免整帧显示耗时延迟舵机更新。
            float tmp_duty = servo_motor_duty_middle - mid_err * 0.5 * SERVO_DIR;
            tmp_duty = MAX(tmp_duty,MIN(SERVO_MOTOR_L_MAX,SERVO_MOTOR_R_MAX));
            tmp_duty = MIN(tmp_duty,MAX(SERVO_MOTOR_L_MAX,SERVO_MOTOR_R_MAX));
            // 退出时过滤不稳定边线，并对连续输出限速；转向接稳后才恢复直道加速。
#if ZEBRA_ENABLE
            tmp_duty = zebra_lane_steering(&zebra_lane, tmp_duty,
                zebra_lane.current, passing && !stopped,
                now, last_servo_angle, servo_motor_duty_middle);
#endif
            if (!stopped &&
                (track_nav.element == TRACK_ROUND_LEFT || track_nav.element == TRACK_ROUND_RIGHT) &&
                track_nav.ring_turn_count >= 2u &&
                (track_nav.phase == NAV_EXIT || track_nav.phase == NAV_RECOVER ||
                 (track_nav.phase == NAV_SLOW &&
                  (track_nav.slow_resume_phase == NAV_EXIT ||
                   track_nav.slow_resume_phase == NAV_RECOVER))) &&
                (track_nav.exit_path_frames < NAV_RING_EXIT_STRAIGHT_FRAMES ||
                 (int32_t)(now-track_nav.exit_turn_until_ms) < 0))
            {
                // 第二次同向转弯至少保持2秒，并持续到近远出口路径均连续有效；再恢复视觉循迹。
                tmp_duty = servo_motor_duty_middle +
                    (track_nav.element == TRACK_ROUND_LEFT ? NAV_RING_EXIT_TURN_ANGLE : -NAV_RING_EXIT_TURN_ANGLE);
                tmp_duty = MAX(tmp_duty, MIN(SERVO_MOTOR_L_MAX, SERVO_MOTOR_R_MAX));
                tmp_duty = MIN(tmp_duty, MAX(SERVO_MOTOR_L_MAX, SERVO_MOTOR_R_MAX));
            }
            if (stopped) tmp_duty = ZEBRA_STRAIGHT_ANGLE;
            tmp_duty = brick_steering(&brick_servo, tmp_duty, brick_owns(&brick),
                brick.valid, now, servo_motor_duty_middle);
            last_servo_angle = tmp_duty;
            pwm_set_duty(SERVO_MOTOR_PWM, SERVO_MOTOR_DUTY(tmp_duty));

#if WIRELESS_SPEED_ENABLE
            // Report accepted navigation separately from unconfirmed candidates.
            const char *road_state = "UNKNOWN";
            if (stopped) road_state = "STOP";
            else if (passing) road_state = "ZEBRA";
            else if (brick_owns(&brick)) road_state=(brick.side?brick.side:brick.candidate)==1?"BRICK_L":"BRICK_R";
            else if (track_nav.phase != NAV_NORMAL && track_nav.element == TRACK_CROSS)
                road_state = "CROSS";
            else if (track_nav.phase != NAV_NORMAL && track_nav.element == TRACK_ROUND_LEFT)
                road_state = "ROUND_L";
            else if (track_nav.phase != NAV_NORMAL && track_nav.element == TRACK_ROUND_RIGHT)
                road_state = "ROUND_R";
            else
            {
                uint8_t shape = track_road_shape(left_line, right_line, MT9V03X_H, MT9V03X_W);
                if (shape) road_state = shape == 1 ? "STRAIGHT" : "CURVE";
            }
            char nav_extra[448];
            ring_side_t *vl=&track_nav.vision.side[0], *vr=&track_nav.vision.side[1];
            snprintf(nav_extra,sizeof nav_extra,
                " F=%u OK=%s V=%u seen=%u/%u feat=%u xf=%u sf=%u xfb=%u age=%lu rs=%u rt=%u Lfeat=%u/%u A=%u/%u/%u B=%u/%u/%u C=%u/%u/%u Rfeat=%u/%u A=%u/%u/%u B=%u/%u/%u C=%u/%u/%u width=%u/%u th=%u eg=%u/%u/%u/%u ap=%u/%u,%u xg=%u/%u/%u/%u/%u xe=%u/%d ctl=%u/%u e10=%d s10=%d rh=%u ep=%u zr=%u zc=%u zp=%u zh=%u/%u zl=%u zmid=%u/%u zpass=%u",
                (unsigned)track_nav.fault,track_element_name(track_detector.confirmed),
                (unsigned)track_detector.ring_votes,(unsigned)appr_seen,(unsigned)enter_seen,
                (unsigned)track_nav.feature_frames,(unsigned)track_nav.exit_confirm_frames,
                (unsigned)track_nav.straight_frames,(unsigned)track_nav.exit_straight_fallback,
                (unsigned long)(now-track_nav.stage_ms),(unsigned)track_nav.slow_resume_phase,
                (unsigned)track_nav.ring_turn_count,
                (unsigned)vl->lost,(unsigned)vl->monotonic,
                (unsigned)vl->a.valid,(unsigned)vl->a.x,(unsigned)vl->a.y,
                (unsigned)vl->b.valid,(unsigned)vl->b.x,(unsigned)vl->b.y,
                (unsigned)vl->c.valid,(unsigned)vl->c.x,(unsigned)vl->c.y,
                (unsigned)vr->lost,(unsigned)vr->monotonic,
                (unsigned)vr->a.valid,(unsigned)vr->a.x,(unsigned)vr->a.y,
                (unsigned)vr->b.valid,(unsigned)vr->b.x,(unsigned)vr->b.y,
                (unsigned)vr->c.valid,(unsigned)vr->c.x,(unsigned)vr->c.y,
                (unsigned)track_nav.width[near_row],(unsigned)track_nav.width[far_row],(unsigned)threshold,
                (unsigned)track_nav.entry_gap_valid,(unsigned)track_nav.entry_gap_rows,
                (unsigned)track_nav.entry_edge_rows,(unsigned)track_nav.entry_guided_rows,
                (unsigned)track_nav.apex_valid,(unsigned)track_nav.apex_x,(unsigned)track_nav.apex_y,
                (unsigned)track_nav.exit_guide_valid,(unsigned)track_nav.exit_guide_row,
                (unsigned)track_nav.exit_guide_center,(unsigned)track_nav.exit_anchor,
                (unsigned)track_nav.exit_reject_rows,
                (unsigned)track_nav.exit_evidence,(int)track_nav.exit_guide_slope,
                (unsigned)control_mid[near_row],(unsigned)control_mid[far_row],
                (int)(mid_err*10.0f),(int)(tmp_duty*10.0f),
                (unsigned)track_nav.ring_edge_turn_hint,(unsigned)track_nav.exit_path_frames,
                (unsigned)zebra_raw_candidate,(unsigned)zebra_candidate,(unsigned)zebra.phase,
                (unsigned)zebra.hits,(unsigned)zebra.clear,
                (unsigned)zebra_lane.current,
                (unsigned)control_mid[near_row],(unsigned)control_mid[far_row],
                (unsigned)passing);
            Wireless_Nav_Update(nav_extra);
            Wireless_Path_Update(track_nav.path_valid[near_row],track_nav.path_valid[far_row],
                track_nav.learned[near_row],track_nav.learned[far_row],track_nav.drive_valid);
            Wireless_Guide_Update(control_mid[near_row],control_mid[far_row],
                track_nav.phase==NAV_ENTER || track_nav.phase==NAV_APPROACH ? track_nav.entry_guide : 0);
            Wireless_Apex_Update(track_nav.apex_valid, track_nav.apex_x, track_nav.apex_y);
            Wireless_Brick_Update((uint8)brick.phase,brick.side,brick.travel);
            Wireless_State_Update(road_state, brick_owns(&brick)?brick_name(&brick):track_nav_name(track_nav.phase),
                track_element_name(track_detector.candidate), (uint8)(ZEBRA_ENABLE ? zebra_race.phase : 255),
                (uint8)(speed_limit <= 0.0f), now);
#endif
#if WIRELESS_IMAGE_ENABLE
            Wireless_Image_Commit(left_line, control_mid, right_line);
#endif
            for (uint8_t _i = 0; _i < MT9V03X_H; ++_i)
            {
                // 将边界线也显示出来
                image[_i][left_line[_i]] = 0;
                image[_i][control_mid[_i]] = 0;
                image[_i][control_left[_i]] = 0;
                image[_i][control_right[_i]] = 0;
                image[_i][right_line[_i]] = 0;
            }
            // 显示图像
            tft180_displayimage03x((uint8_t *)image, 125, 80);
            char status_row[22];
            snprintf(status_row, sizeof(status_row), "S:%-5.5s C:%-5.5s      ",
                stopped ? "STOP" : (brick_owns(&brick)?brick_name(&brick):track_nav_name(track_nav.phase)),
                brick_owns(&brick)?((brick.side?brick.side:brick.candidate)==1?"BRK-L":"BRK-R"):
                    track_element_name(track_detector.candidate));
            tft180_show_string(0, 82, status_row);
#if ZEBRA_ENABLE
            tft180_show_string(0, 92, zebra_status(&zebra, &zebra_race, zebra_candidate));
#else
            tft180_show_string(0, 92, "Z:OFF                ");
#endif

            // 图像占用 y=0..79；识别状态及数字在下方，6x8 字体。
            // 保留完整 int16 位数，避免三位显示截断真实异常值。
            tft180_show_string(0, 102, "L:");
            tft180_show_int(36, 102, encoder_data_1, 5);
            tft180_show_string(0, 112, "R:");
            tft180_show_int(36, 112, encoder_data_2, 5);
#if WIRELESS_SPEED_ENABLE
            Wireless_Speed_Show();
#elif WIRELESS_IMAGE_ENABLE
            Wireless_Image_Show();
#elif WIRELESS_TEST_ENABLE
            Wireless_Test_Show();
#else
            tft180_show_string(0, 122, "LMAX:");
            tft180_show_int(36, 122, encoder_left_max, 5);
#endif
            char ring_row[22];
            int side=track_nav.element==TRACK_ROUND_RIGHT || track_detector.candidate==TRACK_ROUND_RIGHT;
            ring_side_t *v=&track_nav.vision.side[side];
            snprintf(ring_row,sizeof ring_row,"%c A%u B%u C%u F%u       ",side?'R':'L',
                (unsigned)v->a.valid,(unsigned)v->b.valid,(unsigned)v->c.valid,(unsigned)track_nav.fault);
            tft180_show_string(0,132,ring_row);
            snprintf(ring_row,sizeof ring_row,"AP%u EN%u AS%u ES%u      ",
                (unsigned)(track_nav.phase==NAV_APPROACH),(unsigned)(track_nav.phase==NAV_ENTER),
                (unsigned)appr_seen,(unsigned)enter_seen);
            tft180_show_string(0,142,ring_row);
            tft180_show_string(0, 152, "TGT:");
            tft180_show_float(36, 152, speed_target, 2, 1);

            // 处理完一帧图像后务必把该标志位清零！
            mt9v03x_finish_flag = 0;
        }
#if WIRELESS_SPEED_ENABLE
        Wireless_Speed_Poll();
#elif WIRELESS_IMAGE_ENABLE
        Wireless_Image_Poll();
#elif WIRELESS_TEST_ENABLE
        Wireless_Test_Poll();
#endif
    }
}
