#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef int16_t int16;
#define WIRELESS_UART_INDEX 8
#define WIRELESS_UART_RTS_PIN 26
#define LPUART8 8
#define kLPUART_TxDataRegEmptyFlag 1
#define __DMB() ((void)0)
static uint32 control_time_ms, irq_mask, speed_distance_counts;
static float speed_target;
static int16 encoder_data_1, encoder_data_2;
static uint8 actual[288], expected[288];
static unsigned used;
static int busy, enabled, capacity;
static uint32 __get_PRIMASK(void) { return irq_mask; }
static void __disable_irq(void) { irq_mask = 1; }
static void __set_PRIMASK(uint32 m) { irq_mask = m; }
static int gpio_get_level(int pin) { (void)pin; return busy; }
static void uart_tx_interrupt(int uart, int e) { (void)uart; enabled = e; }
static int LPUART_GetStatusFlags(int uart) { (void)uart; return capacity > 0; }
static void LPUART_WriteByte(int uart, uint8 b)
{
    (void)uart;
    assert(!busy && capacity > 0 && used < 288);
    actual[used++] = b;
    --capacity;
}
static void tft180_show_string(int x, int y, char *s)
{ (void)x; (void)y; assert(strlen(s) == 21); }
#include "wireless_speed.h"
static void check_frame(float target, int16 left, int16 right)
{
    unsigned expected_length;
    Wireless_State_Update("ROUND_L", "ENTER", "LEFT", 1, 0, control_time_ms);
    expected_length = (unsigned)snprintf((char *)expected, sizeof(expected),
        "ms=%lu tgt=%.1f L=%d R=%d avg=%.1f road=ROUND_L nav=ENTER cand=LEFT zebra=ARMED blocked=0 stale=0 arc=0:0,0 path=00 learn=00 drive=0 mid=0,0 entry=0 brick=0 bside=0 bdist=0 odo=0\r\n",
        (unsigned long)control_time_ms, (double)target, (int)left, (int)right,
        ((double)left + right) * 0.5);
    speed_target = target; encoder_data_1 = left; encoder_data_2 = right;
    used = 0; capacity = 0;
    Wireless_Speed_Poll();
    assert(enabled && wireless_speed_active && irq_mask == 0);
    assert(memcmp(expected, wireless_speed_packet, expected_length) == 0);
    Wireless_Speed_Tx_IRQHandler(); // UART full cannot block.
    assert(used == 0);
    capacity = 3;
    Wireless_Speed_Tx_IRQHandler();
    assert(used == 3);
    encoder_data_1 = 1234; speed_target = 1000; // Immutable in-flight sample.
    capacity = 20; // TX-ready interrupt after FIFO drains.
    busy = 1;
    Wireless_Speed_Tx_IRQHandler();
    Wireless_Speed_Poll();
    assert(used == 3 && !enabled);
    busy = 0;
    Wireless_Speed_Poll();
    capacity = 256;
    while (wireless_speed_active)
    {
        unsigned before = used;
        Wireless_Speed_Tx_IRQHandler();
        assert(used - before <= 30);
    }
    assert(!enabled && used == expected_length);
    assert(memcmp(expected, actual, expected_length) == 0);
}

int main(void)
{
    control_time_ms = 100;
    Wireless_Speed_Poll(); assert(!enabled); // Init failed.
    wireless_speed_ready = 1;
    control_time_ms = 99;
    Wireless_Speed_Poll(); assert(!enabled);
    control_time_ms = 100;
    check_frame(70.0f, 66, 69);
    Wireless_Speed_Poll(); assert(!enabled); // Rate limited.
    control_time_ms = 199;
    Wireless_Speed_Poll(); assert(!enabled);
    control_time_ms = 200;
    check_frame(0.0f, -12, 5); // Coasting/reverse preserved with zero target.
    control_time_ms = 300;
    check_frame(45.5f, INT16_MIN, INT16_MAX); // Signed range and average.
    wireless_speed_last_ms = UINT32_MAX - 49;
    control_time_ms = 50;
    check_frame(0, 0, 0); // Rollover, stopped vehicle.
    assert(wireless_speed_sent == 4);
    enabled = 1;
    Wireless_Speed_Tx_IRQHandler(); assert(!enabled); // Completion race.
    Wireless_Apex_Update(1,60,40);
    Wireless_Guide_Update(82,94,1);
    control_time_ms += 100;
    Wireless_State_Update("ROUND_L","ENTER","LEFT",1,0,control_time_ms);
    Wireless_Speed_Poll();
    assert(strstr((char *)wireless_speed_packet,"arc=1:60,40 path=00 learn=00 drive=0 mid=82,94 entry=1 brick=0 bside=0 bdist=0 odo=0\r\n") != NULL);
    used=0; capacity=256;
    while(wireless_speed_active) Wireless_Speed_Tx_IRQHandler();
    control_time_ms += 400;
    Wireless_Speed_Poll();
    assert(strstr((char *)wireless_speed_packet, "road=UNKNOWN") != NULL);
    assert(strstr((char *)wireless_speed_packet, "stale=1 arc=0:0,0 path=00 learn=00 drive=0 mid=82,94 entry=0 brick=0 bside=0 bdist=0 odo=0\r\n") != NULL);
    used=0; capacity=256;
    while(wireless_speed_active) Wireless_Speed_Tx_IRQHandler();
    control_time_ms += 100;
    Wireless_State_Update("STRAIGHT","RUN","NONE",255,0,control_time_ms);
    Wireless_Speed_Poll();
    assert(strstr((char *)wireless_speed_packet,"zebra=OFF ") != NULL);
    used=0; capacity=288;
    while(wireless_speed_active) Wireless_Speed_Tx_IRQHandler();
    control_time_ms=UINT32_MAX;
    speed_distance_counts=UINT32_MAX;
    Wireless_Brick_Update(2,2,UINT32_MAX);
    Wireless_State_Update("BRICK_R","HOLD ","UNKNOWN",255,0,control_time_ms);
    encoder_data_1=encoder_data_2=INT16_MIN;
    Wireless_Speed_Poll();
    assert(wireless_speed_active && wireless_speed_length<288);
    assert(strstr((char *)wireless_speed_packet,"brick=2 bside=2 bdist=4294967295 odo=4294967295\r\n"));
    used=0; capacity=288;
    while(wireless_speed_active) Wireless_Speed_Tx_IRQHandler();
    Wireless_Speed_Show();
    puts("PASS: ASCII states and stale detection, 10Hz, raw signed encoders, snapshots, RTS resume, no busy wait, rollover");
    return 0;
}
