#ifndef SMARTCAR_WIRELESS_SPEED_H
#define SMARTCAR_WIRELESS_SPEED_H
#include <string.h>

// ASCII telemetry at 10Hz. Speed unit: encoder counts / 10ms.
#define WIRELESS_SPEED_PERIOD_MS 100u
#define WIRELESS_SPEED_BYTES 256u
static uint8 wireless_speed_packet[WIRELESS_SPEED_BYTES];
static uint8 wireless_speed_ready;
static volatile uint8 wireless_speed_active;
static volatile uint16 wireless_speed_pos, wireless_speed_length;
static const char *wireless_road = "UNKNOWN", *wireless_nav = "NORMAL";
static const char *wireless_candidate = "NONE";
static uint8 wireless_zebra, wireless_blocked, wireless_state_ready;
static uint32 wireless_state_ms;

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
    uint32 now, mask;
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
    __set_PRIMASK(mask);
    stale = !wireless_state_ready || (uint32)(now - wireless_state_ms) >= 300u;
    length = snprintf((char *)wireless_speed_packet, sizeof(wireless_speed_packet),
        "ms=%lu tgt=%.1f L=%d R=%d avg=%.1f road=%s nav=%s cand=%s zebra=%s blocked=%u stale=%u\r\n",
        (unsigned long)now, (double)target, (int)left, (int)right,
        ((double)left + (double)right) * 0.5,
        stale ? "UNKNOWN" : wireless_road, wireless_nav, wireless_candidate,
        wireless_zebra < 5 ? zebra_names[wireless_zebra] : "UNKNOWN",
        (unsigned)wireless_blocked, (unsigned)stale);
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
