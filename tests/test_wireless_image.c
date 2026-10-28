#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
#define MT9V03X_W 188
#define MT9V03X_H 120
#define WIRELESS_UART_INDEX 8
#define WIRELESS_UART_RTS_PIN 26
#define LPUART8 8
#define kLPUART_TxDataRegEmptyFlag 1
#define __DMB() ((void)0)
static uint32 control_time_ms;
static int busy, tx_enabled, writable = 1, write_capacity = 100000;
static uint8 actual[50000], expected[50000];
static unsigned actual_size, expected_size;
static int gpio_get_level(int pin) { (void)pin; return busy; }
static void uart_tx_interrupt(int uart, int enable) { (void)uart; tx_enabled = enable; }
static int LPUART_GetStatusFlags(int uart) { (void)uart; return writable && write_capacity > 0; }
static void LPUART_WriteByte(int uart, uint8 byte)
{
    (void)uart;
    assert(!busy && write_capacity > 0 && actual_size < sizeof(actual));
    actual[actual_size++] = byte;
    --write_capacity;
}
static void tft180_show_string(int x, int y, char *s)
{
    (void)x; (void)y;
    assert(strlen(s) == 21);
}
#include "wireless_image.h"
#include "reference.h"
static detector_camera_struct detector_camera_data;
static detector_camera_dot_struct detector_camera_dot_data;
static detector_camera_buffer_struct detector_camera_buffer;
static int detector_transfer_type = DETECTOR_CUSTOM;
typedef struct { uint8 *source[17]; uint32 length[17]; } wifi_spi_send_multi_struct;
static void wifi_spi_send_buffer_multi(void *p) { (void)p; assert(0); }
#define zf_assert assert
static void detector_transfer(const void *p, uint32 n)
{
    assert(expected_size + n <= sizeof(expected));
    memcpy(expected + expected_size, p, n);
    expected_size += n;
}
#include "reference.inc"

int main(void)
{
    uint8 gray[MT9V03X_H * MT9V03X_W], left[120], mid[120], right[120];
    uint8 preview[96 * 60], small_left[60], small_mid[60], small_right[60];
    unsigned x, y, i, before;
    for (i = 0; i < sizeof(gray); ++i) gray[i] = (uint8)(i * 19u);
    for (i = 0; i < 120; ++i) { left[i] = (uint8)(i / 4); mid[i] = 94; right[i] = 180; }
    right[118] = 255; // Invalid source coordinate must stay inside preview.
    for (y = 0; y < 60; ++y)
    {
        for (x = 0; x < 96; ++x) preview[y * 96 + x] = gray[y * 2 * 188 + x * 188 / 96];
        small_left[y] = (uint8)(left[y * 2] * 96u / 188u);
        small_mid[y] = (uint8)(mid[y * 2] * 96u / 188u);
        small_right[y] = (uint8)((right[y * 2] < 188 ? right[y * 2] : 187u) * 96u / 188u);
    }
    detector_camera_information_config(DETECTOR_GRAY, preview, 96, 60);
    detector_camera_boundary_config(X_BOUNDARY, 60, small_left, small_mid, small_right, NULL, NULL, NULL);
    detector_camera_send();
    assert(expected_size == 5956);

    control_time_ms = 1000;
    Wireless_Image_Begin(gray); // Uninitialized radio must not publish.
    assert(!wireless_image_capturing);
    wireless_image_ready = 1;
    control_time_ms = 99;
    Wireless_Image_Begin(gray);
    assert(!wireless_image_capturing);
    control_time_ms = 1000;
    Wireless_Image_Begin(gray);
    Wireless_Image_Poll();
    assert(!tx_enabled && !wireless_image_active);
    memset(gray, 0, sizeof(gray)); // Binary conversion cannot alter snapshot.
    Wireless_Image_Commit(left, mid, right);
    assert(memcmp(expected, wireless_image_packet, expected_size) == 0);
    memset(left, 0, sizeof(left));
    memset(mid, 0, sizeof(mid));
    memset(right, 0, sizeof(right));
    control_time_ms = 5000;
    Wireless_Image_Begin(gray); // Backpressure: don't replace in-flight frame.
    Wireless_Image_Commit(left, mid, right);
    assert(memcmp(expected, wireless_image_packet, expected_size) == 0);

    busy = 1;
    Wireless_Image_Poll();
    assert(!tx_enabled);
    Wireless_Image_Tx_IRQHandler();
    assert(!actual_size && !tx_enabled);
    busy = 0;
    Wireless_Image_Poll();
    assert(tx_enabled);
    writable = 0;
    Wireless_Image_Tx_IRQHandler(); // Full UART returns without waiting.
    assert(!actual_size);
    writable = 1;
    write_capacity = 1;
    Wireless_Image_Tx_IRQHandler();
    assert(actual_size == 1); // FIFO capacity rechecked on each byte.
    write_capacity = 100000;
    Wireless_Image_Tx_IRQHandler();
    assert(actual_size == 31); // ISR has bounded work.
    busy = 1;
    Wireless_Image_Tx_IRQHandler();
    assert(actual_size == 31 && !tx_enabled);
    busy = 0;
    Wireless_Image_Poll();
    while (wireless_image_active)
    {
        before = actual_size;
        Wireless_Image_Tx_IRQHandler();
        assert(actual_size - before <= 30);
    }
    assert(!tx_enabled && wireless_image_sent == 1);
    assert(actual_size == expected_size && memcmp(actual, expected, expected_size) == 0);
    tx_enabled = 1; // Simulate poll/ISR completion race.
    Wireless_Image_Tx_IRQHandler();
    assert(!tx_enabled && actual_size == expected_size);
    Wireless_Image_Show();
    wireless_image_last_ms = UINT32_MAX - 500;
    control_time_ms = 600;
    Wireless_Image_Begin(gray); // Clock wrap and next-frame reuse.
    assert(wireless_image_capturing);
    Wireless_Image_Commit(left, mid, right);
    assert(wireless_image_active && wireless_image_pos == 0);
    Wireless_Image_Show();
    puts("PASS: official protocol parity, snapshots, flow control, bounded ISR, completion race, clock wrap");
    return 0;
}
