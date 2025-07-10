/*
 * Copyright (c) 2024 Open Pixel Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/spi.h>
#include <nrfx_spis.h>





#include <zephyr/drivers/uart.h>

#include <string.h>



#define SPIOP (SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_SLAVE)




#define IMG_WIDTH      160
#define IMG_HEIGHT     120




#define PIN_VSYNC  2   /* FVLD  */
#define PIN_HSYNC  3   /* LVLD  */
#define PIN_PCLK   12   /* Pixel clock / SPIS SCK */




#define PWM_LED0    DT_ALIAS(arg)

#define I2C_NODE DT_ALIAS(hmm)


static uint8_t image[IMG_WIDTH * IMG_HEIGHT] __aligned(4);

struct spi_dt_spec spispec = SPI_DT_SPEC_GET(DT_NODELABEL(gendev), SPIOP, 0);



// HM01B0 register addresses
//#define REG_MODEL_ID_H    0x0000
//#define REG_MODEL_ID_L    0x0001
//#define REG_MODE_SELECT   0x0100
//#define REG_BIT_CONTROL   0x3059



void capture_frame(const struct device *gpio0_dev) {
    memset(image, 0, sizeof(image));
    int y = 0;

    /* ------------------------------------------------------------------
     * Frame synchronisation
     * ------------------------------------------------------------------
     * 1)  Wait for the sensor to finish the *current* frame (VSYNC LOW).
     * 2)  Wait for the rising edge – marks the beginning of the very next
     *     frame so that our line counter starts at 0.
     */

    while (gpio_pin_get(gpio0_dev, PIN_VSYNC) == 1);
    while (gpio_pin_get(gpio0_dev, PIN_VSYNC) == 0);

    y = 0;

    while ((y < IMG_HEIGHT) && (gpio_pin_get(gpio0_dev, PIN_VSYNC) == 1)) {

        while (gpio_pin_get(gpio0_dev, PIN_HSYNC) == 1) {
            if (gpio_pin_get(gpio0_dev, PIN_VSYNC) == 0)
                goto frame_done;
        }

        static uint8_t tx_dummy_line[IMG_WIDTH];

        struct spi_buf tx_buf_line = {
            .buf = tx_dummy_line,
            .len = IMG_WIDTH,
        };
        const struct spi_buf_set tx_set_line = {
            .buffers = &tx_buf_line,
            .count   = 1,
        };

        struct spi_buf rx_buf_line = {
            .buf = &image[y * IMG_WIDTH],
            .len = IMG_WIDTH,
        };
        const struct spi_buf_set rx_set_line = {
            .buffers = &rx_buf_line,
            .count   = 1,
        };
        //__DMB();
        int err = spi_transceive_dt(&spispec, &tx_set_line, &rx_set_line);
        
        if (err < 0) {
            memset(rx_buf_line.buf, 0, rx_buf_line.len);
        }

        /* Wait for LVLD to de-assert (HIGH→LOW).  If FVLD happens to drop
         * while we are waiting, break out so we do not continue into the
         * blanking period or the next frame. */
        while (gpio_pin_get(gpio0_dev, PIN_HSYNC) == 1) {
            if (gpio_pin_get(gpio0_dev, PIN_VSYNC) == 0) {
                break;
            }
        }

        y++;
    }

frame_done:
    /* If, for any reason, we captured fewer than IMG_HEIGHT lines (e.g. the
     * sensor sent fewer lines than expected), clear the remainder so that
     * the host always receives a well-formed 160×120 frame. */
    while (y < IMG_HEIGHT) {
        memset(&image[y * IMG_WIDTH], 0, IMG_WIDTH);
        y++;
    }

    printk("Frame captured! (120 lines)\n");
}

static const struct device *gpio0_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));




static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);


// UART transmitting code

#define UART_NODE DT_NODELABEL(uart0)
static const struct device *uart_dev;


/**
 * Send a complete frame over UART in hexadecimal ASCII format.
 * Each pixel byte is encoded as two hex characters, and rows are separated by newlines.
 * Returns 0 on success or negative on error.
 */
static void send_frame_over_uart_hex(void)
{
    static const char frame_start[] = "<FRAME>\n";
    static const char frame_end[]   = "</FRAME>\n";
    static const char hex_chars[]   = "0123456789ABCDEF";

    for (size_t i = 0; i < sizeof(frame_start) - 1; i++) {
        uart_poll_out(uart_dev, frame_start[i]);
    }
    for (int y = 0; y < IMG_HEIGHT; y++) {
        for (int x = 0; x < IMG_WIDTH; x++) {
            uint8_t pixel = image[y * IMG_WIDTH + x];
            uart_poll_out(uart_dev, hex_chars[(pixel >> 4) & 0x0F]);
            uart_poll_out(uart_dev, hex_chars[pixel & 0x0F]);
        }
        uart_poll_out(uart_dev, '\n');
    }
    for (size_t i = 0; i < sizeof(frame_end) - 1; i++) {
        uart_poll_out(uart_dev, frame_end[i]);
    }
}


void capture_and_send_frame(const struct device *gpio0_dev)
{
    capture_frame(gpio0_dev);
    send_frame_over_uart_hex();
}
int main(void)
{



    /* ---------- UART first ---------- */
    uart_dev = DEVICE_DT_GET(UART_NODE);
    if (!device_is_ready(uart_dev)) {
        printk("UART device not ready!\n");
        return -1;
    }
    uart_fifo_fill(uart_dev, (const uint8_t *)"HELLO UART!\n", 12);

    /* ---------- SPI sanity ---------- */
    if (!device_is_ready(spispec.bus)) {
        printk("SPI device not ready!\n");
        return -1;
    }

    /* ---------- I2C sanity ---------- */
    if (!device_is_ready(dev_i2c.bus)) {
        printk("I2C bus %s is not ready!\n", dev_i2c.bus->name);
        return -1;
    }

    /* ---------- Read chip-ID ---------- */
    uint8_t id_h = 0, id_l = 0;
    uint8_t addr[2] = {0x00, 0x00};
    i2c_write_read_dt(&dev_i2c, addr, 2, &id_h, 1);
    addr[1] = 0x01;
    i2c_write_read_dt(&dev_i2c, addr, 2, &id_l, 1);
    printk("HM01B0 ID: 0x%02X%02X\n", id_h, id_l);

    /* ---------- Register block ---------- */
    const uint8_t cfg[][3] = {
        {0x01,0x04,0x01},  /* 0: GROUP_HOLD start                 */

        /* -------------------------------- Video window ------------------- */
        {0x30,0x10,0x01},  /* 1: QVGA window                      */

        /* -------------------------------- Misc ISP ---------------------- */
        {0x03,0x81,0x01},
        {0x03,0x83,0x03},
        {0x03,0x85,0x01},
        {0x03,0x87,0x03},
        {0x03,0x90,0x03},  /* enable 2×2 binning                  */

        /* -------------------------------- IO format --------------------- */
        {0x30,0x11,0x00},  /* 7: DATAFORMAT – RAW8 (bit0 = 0)     */
        {0x30,0x59,0x20},  /* 8: BIT_CONTROL – 1-bit serial      */
        {0x30,0x60,0x00},  /* 9: OSC_CLK_DIV – MSB-first, ÷8    */

        /* -------------------------------- Disable test pattern --------- */
        {0x06,0x01,0x00},  /* 10: TEST_PATTERN_MODE – disable     */

        /* -------------------------------- Manual exposure -------------- */
        /* Use modest integration time (0x0040 lines) and unity analogue gain. */
        {0x02,0x04,0x01},
        {0x01,0x04,0x00},  /* 11: GROUP_HOLD apply                */
        {0x01,0x00,0x01},  /* 12: MODE_SELECT – streaming         */
    };
    for (int i = 0; i < ARRAY_SIZE(cfg); i++)
        i2c_write_dt(&dev_i2c, cfg[i], 3);

    k_sleep(K_MSEC(10));

    /* ------ Sanity-check a couple of critical registers -------- */
    {
        uint8_t addr_readback[2];
        uint8_t val;

        /* BIT_CONTROL should read back 0x20 for 1-bit serial mode. */
        addr_readback[0] = 0x30; addr_readback[1] = 0x59;
        if (i2c_write_read_dt(&dev_i2c, addr_readback, 2, &val, 1) == 0)
            printk("BIT_CONTROL (0x3059) = 0x%02X\n", val);

        /* OSC_CLK_DIV should read back 0x00 (MSB-first, ÷8, non-gated). */
        addr_readback[0] = 0x30; addr_readback[1] = 0x60;
        if (i2c_write_read_dt(&dev_i2c, addr_readback, 2, &val, 1) == 0)
            printk("OSC_CLK_DIV (0x3060) = 0x%02X\n", val);

        /* TEST_PATTERN_MODE – should be 0x00 (disabled) */
        addr_readback[0] = 0x06; addr_readback[1] = 0x01;
        if (i2c_write_read_dt(&dev_i2c, addr_readback, 2, &val, 1) == 0)
            printk("TEST_PATTERN (0x0601) = 0x%02X\n", val);
    }

    

    gpio_pin_configure(gpio0_dev, PIN_VSYNC, GPIO_INPUT);
    gpio_pin_configure(gpio0_dev, PIN_HSYNC, GPIO_INPUT);


    while (1) {
        capture_and_send_frame(gpio0_dev);
    }
}
