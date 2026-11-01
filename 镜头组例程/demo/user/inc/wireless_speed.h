#ifndef SMARTCAR_WIRELESS_SPEED_H
#define SMARTCAR_WIRELESS_SPEED_H
#include <string.h>

// ASCII telemetry at 10Hz. Speed unit: encoder counts / 10ms.
#define WIRELESS_SPEED_PERIOD_MS 100u
#define WIRELESS_SPEED_BYTES 768u
static char wireless_nav_extra[448];
static void Wireless_Nav_Update(const char *text)
{ snprintf(wireless_nav_extra,sizeof wireless_nav_extra,"%s",text); }
static uint8 wireless_speed_packet[WIRELESS_SPEED_BYTES];
static uint8 wireless_speed_ready;
static volatile uint8 wireless_speed_active;
static volatile uint16 wireless_speed_pos, wireless_speed_length;
static const char *wireless_road = "UNKNOWN", *wireless_nav = "NORMAL";
static const char *wireless_candidate = "NONE";
static uint8 wireless_zebra, wireless_blocked, wireless_state_ready;
static uint32 wireless_state_ms;
static uint8 wireless_path_near,wireless_path_far,wireless_learn_near,wireless_learn_far,wireless_drive;
static uint8 wireless_mid_near,wireless_mid_far,wireless_entry_guide;
static uint8 wireless_brick_phase,wireless_brick_side;
static uint32 wireless_brick_travel;
static void Wireless_Brick_Update(uint8 phase,uint8 side,uint32 travel)
{ wireless_brick_phase=phase;wireless_brick_side=side;wireless_brick_travel=travel; }
static void Wireless_Guide_Update(uint8 near,uint8 far,uint8 guide)
{ wireless_mid_near=near;wireless_mid_far=far;wireless_entry_guide=guide; }
static void Wireless_Path_Update(uint8 pn,uint8 pf,uint8 ln,uint8 lf,uint8 drive)
{ wireless_path_near=pn;wireless_path_far=pf;wireless_learn_near=ln;wireless_learn_far=lf;wireless_drive=drive; }

static uint8 wireless_arc_valid, wireless_arc_x, wireless_arc_y;
static void Wireless_Apex_Update(uint8 valid, uint8 x, uint8 y)
{ wireless_arc_valid=valid; wireless_arc_x=valid?x:0; wireless_arc_y=valid?y:0; }

static void Wireless_State_Update(const char *road, const char *nav, const char *candidate,
                                  uint8 zebra, uint8 blocked, uint32 now)
{
    wireless_road = road; wireless_nav = nav; wireless_candidate = candidate;
    wireless_zebra = zebra; wireless_blocked = blocked;
    wireless_state_ms = now; wireless_state_ready = 1;
}
static volatile uint16 wireless_speed_sent;
static uint32 wireless_speed_last_ms;

void Wireless_Speed_Tx_IRQHandler(void)
{
    uint8 budget = 30;
    if (!wireless_speed_active)
    {
        uart_tx_interrupt(WIRELESS_UART_INDEX, 0);
        return;
    }
    while (budget-- && (LPUART_GetStatusFlags(LPUART8) & kLPUART_TxDataRegEmptyFlag))
    {
        if (gpio_get_level(WIRELESS_UART_RTS_PIN))
        {
            uart_tx_interrupt(WIRELESS_UART_INDEX, 0);
            return;
        }
        LPUART_WriteByte(LPUART8, wireless_speed_packet[wireless_speed_pos++]);
        if (wireless_speed_pos == wireless_speed_length)
        {
            uart_tx_interrupt(WIRELESS_UART_INDEX, 0);
            if (wireless_speed_sent < 65535u) ++wireless_speed_sent;
            __DMB();
            wireless_speed_active = 0;
            return;
        }
    }
}

static void Wireless_Speed_Poll(void)
{
    uint32 now, mask, odom;
    int16 left, right;
    float target;
    int length;
    uint8 stale;
    static const char * const zebra_names[] = {"START", "ARMED", "PASS", "CLEAR", "STOP"};
    if (!wireless_speed_ready) return;
    if (wireless_speed_active)
    {
        if (!gpio_get_level(WIRELESS_UART_RTS_PIN))
            uart_tx_interrupt(WIRELESS_UART_INDEX, 1);
        return;
    }
    if ((uint32)(control_time_ms - wireless_speed_last_ms) < WIRELESS_SPEED_PERIOD_MS ||
        gpio_get_level(WIRELESS_UART_RTS_PIN)) return;

    // A few loads only: get both encoder readings and target from one PIT tick.
    // Copy raw encoder counts even when target is zero (coasting remains visible).
    mask = __get_PRIMASK();
    __disable_irq();
    now = control_time_ms;
    target = speed_target;
    left = encoder_data_1;
    right = encoder_data_2;
    odom = speed_distance_counts;
    __set_PRIMASK(mask);
    stale = !wireless_state_ready || (uint32)(now - wireless_state_ms) >= 300u;
    length = snprintf((char *)wireless_speed_packet, sizeof(wireless_speed_packet),
        "ms=%lu tgt=%.1f L=%d R=%d avg=%.1f road=%s nav=%s cand=%s zebra=%s blocked=%u stale=%u arc=%u:%u,%u path=%u%u learn=%u%u drive=%u mid=%u,%u entry=%u brick=%u bside=%u bdist=%lu odo=%lu\r\n",
        (unsigned long)now, (double)target, (int)left, (int)right,
        ((double)left + (double)right) * 0.5,
        stale ? "UNKNOWN" : wireless_road, wireless_nav, wireless_candidate,
        wireless_zebra == 255 ? "OFF" : (wireless_zebra < 5 ? zebra_names[wireless_zebra] : "UNKNOWN"),
        (unsigned)wireless_blocked, (unsigned)stale,
        (unsigned)(!stale && wireless_arc_valid),
        (unsigned)(stale?0:wireless_arc_x), (unsigned)(stale?0:wireless_arc_y),
        (unsigned)wireless_path_near,(unsigned)wireless_path_far,
        (unsigned)wireless_learn_near,(unsigned)wireless_learn_far,(unsigned)wireless_drive,
        (unsigned)wireless_mid_near,(unsigned)wireless_mid_far,
        (unsigned)(stale?0:wireless_entry_guide),(unsigned)wireless_brick_phase,
        (unsigned)wireless_brick_side,(unsigned long)wireless_brick_travel,(unsigned long)odom);
    if(length>=2 && length<(int)sizeof wireless_speed_packet) {
        int added=snprintf((char *)wireless_speed_packet+length-2,
            sizeof wireless_speed_packet-(length-2),"%s\r\n",wireless_nav_extra);
        if(added<0) return;
        length=length-2+added;
    }
    if (length <= 0 || (unsigned)length >= sizeof(wireless_speed_packet)) return;
    wireless_speed_length = (uint16)length;
    wireless_speed_last_ms = now;
    wireless_speed_pos = 0;
    __DMB();
    wireless_speed_active = 1;
    uart_tx_interrupt(WIRELESS_UART_INDEX, 1);
}

static void Wireless_Speed_Show(void)
{
    char row[22];
    snprintf(row, sizeof(row), "V%u R%u T%05u 10Hz    ",
        (unsigned)wireless_speed_ready,
        (unsigned)(gpio_get_level(WIRELESS_UART_RTS_PIN) != 0),
        (unsigned)wireless_speed_sent);
    tft180_show_string(0, 122, row);
}
#endif
