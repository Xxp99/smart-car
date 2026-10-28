#ifndef SMARTCAR_WIRELESS_IMAGE_H
#define SMARTCAR_WIRELESS_IMAGE_H

#include <string.h>

// Mirrors zf_device_detector: grayscale image then three X-only boundaries.
// One immutable packet; no blocking writes, no partial packet replacement.
// Width is a multiple of four for grayscale bitmap row alignment.
#define WIRELESS_IMAGE_WIDTH 96u
#define WIRELESS_IMAGE_HEIGHT 60u
#define WIRELESS_IMAGE_PIXELS (WIRELESS_IMAGE_WIDTH * WIRELESS_IMAGE_HEIGHT)
#define WIRELESS_IMAGE_BYTES (16u + WIRELESS_IMAGE_PIXELS + 3u * WIRELESS_IMAGE_HEIGHT)
static uint8 wireless_image_packet[WIRELESS_IMAGE_BYTES];
static volatile uint32 wireless_image_pos;
static volatile uint8 wireless_image_active;
static volatile uint16 wireless_image_sent;
static uint8 wireless_image_ready, wireless_image_capturing;
static uint32 wireless_image_last_ms;

static void Wireless_Image_Begin(const uint8 *gray)
{
    uint32 x, y;
    uint32 now = control_time_ms;
    uint8 *p = wireless_image_packet;
    if (!wireless_image_ready || wireless_image_active || wireless_image_capturing ||
        (uint32)(now - wireless_image_last_ms) < 100u)
        return;
    wireless_image_last_ms = now;
    wireless_image_capturing = 1;
    p[0] = 0xAA; p[1] = 0x02; p[2] = 0x43; p[3] = 8;
    p[4] = (uint8)WIRELESS_IMAGE_WIDTH; p[5] = 0;
    p[6] = (uint8)WIRELESS_IMAGE_HEIGHT; p[7] = 0;
    // Nearest-neighbor preview; the control image remains full resolution.
    for (y = 0; y < WIRELESS_IMAGE_HEIGHT; ++y)
        for (x = 0; x < WIRELESS_IMAGE_WIDTH; ++x)
            p[8 + y * WIRELESS_IMAGE_WIDTH + x] =
                gray[(y * MT9V03X_H / WIRELESS_IMAGE_HEIGHT) * MT9V03X_W +
                     x * MT9V03X_W / WIRELESS_IMAGE_WIDTH];
}

static void Wireless_Image_Commit(const uint8 *left, const uint8 *mid, const uint8 *right)
{
    uint32 y, row;
    uint8 *p = wireless_image_packet + 8 + WIRELESS_IMAGE_PIXELS;
    if (!wireless_image_capturing) return;
    p[0] = 0xAA; p[1] = 0x03; p[2] = 3; p[3] = 8;
    p[4] = (uint8)WIRELESS_IMAGE_HEIGHT; p[5] = 0;
    p[6] = 7; p[7] = 0;
    for (y = 0; y < WIRELESS_IMAGE_HEIGHT; ++y)
    {
        row = y * MT9V03X_H / WIRELESS_IMAGE_HEIGHT;
        p[8 + y] = (uint8)((left[row] < MT9V03X_W ? left[row] : MT9V03X_W - 1u) * WIRELESS_IMAGE_WIDTH / MT9V03X_W);
        p[8 + WIRELESS_IMAGE_HEIGHT + y] = (uint8)((mid[row] < MT9V03X_W ? mid[row] : MT9V03X_W - 1u) * WIRELESS_IMAGE_WIDTH / MT9V03X_W);
        p[8 + 2 * WIRELESS_IMAGE_HEIGHT + y] = (uint8)((right[row] < MT9V03X_W ? right[row] : MT9V03X_W - 1u) * WIRELESS_IMAGE_WIDTH / MT9V03X_W);
    }
    wireless_image_pos = 0;
    wireless_image_capturing = 0;
    __DMB(); // Publish only after all image and boundary bytes are ready.
    wireless_image_active = 1;
}

void Wireless_Image_Tx_IRQHandler(void)
{
    uint8 budget = 30;
    if (!wireless_image_active)
    {
        // Poll may have been preempted between checking active and enabling TX.
        uart_tx_interrupt(WIRELESS_UART_INDEX, 0);
        return;
    }
    while (budget-- && (LPUART_GetStatusFlags(LPUART8) & kLPUART_TxDataRegEmptyFlag))
    {
        if (gpio_get_level(WIRELESS_UART_RTS_PIN))
        {
            uart_tx_interrupt(WIRELESS_UART_INDEX, 0);
            return; // Main loop re-arms when RTS becomes low; never spin here.
        }
        LPUART_WriteByte(LPUART8, wireless_image_packet[wireless_image_pos++]);
        if (wireless_image_pos == WIRELESS_IMAGE_BYTES)
        {
            uart_tx_interrupt(WIRELESS_UART_INDEX, 0);
            if (wireless_image_sent < 65535u) ++wireless_image_sent;
            __DMB();
            wireless_image_active = 0;
            return;
        }
    }
}

static void Wireless_Image_Poll(void)
{
    if (wireless_image_ready && wireless_image_active &&
        !gpio_get_level(WIRELESS_UART_RTS_PIN))
        uart_tx_interrupt(WIRELESS_UART_INDEX, 1);
}

static void Wireless_Image_Show(void)
{
    char row[22];
    uint32 pending = wireless_image_active ? WIRELESS_IMAGE_BYTES - wireless_image_pos : 0;
    // F counts packets queued to UART, not acknowledgements from the PC.
    snprintf(row, sizeof(row), "I%u R%u F%05u Q%05lu  ",
        (unsigned)wireless_image_ready,
        (unsigned)(gpio_get_level(WIRELESS_UART_RTS_PIN) != 0),
        (unsigned)wireless_image_sent, (unsigned long)pending);
    tft180_show_string(0, 122, row);
}

#endif
