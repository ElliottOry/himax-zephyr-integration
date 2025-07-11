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

#include "HM01B0Regs.h"





#include <zephyr/drivers/uart.h>

#include <string.h>



#define SPIOP (SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_SLAVE)




#define IMG_WIDTH      160
#define IMG_HEIGHT     120




#define PIN_VSYNC  27   /* FVLD  */
#define PIN_HSYNC  11  /* LVLD  */
#define PIN_PCLK   15   /* Pixel clock / SPIS SCK */




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

    //printk("Frame captured! (120 lines)\n");
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
/*
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
*/
/*
 * Transmit the captured frame over UART.
 *
 * The previous implementation pushed the raw pixel values directly onto the
 * UART.  That resulted in a stream that contains many non-printable ASCII
 * characters; most terminal emulators render those bytes as question marks
 * (�, ?).
 *
 * To make the output human-readable without changing the amount of data that
 * is sent, each pixel is now converted to an 8-character ASCII binary string
 * (MSB first, e.g. 0x5A -> "01011010").  The resulting stream can be viewed
 * in any terminal and parsed reliably by a host script.
 *
 * Format:
 *   <FRAME>\n
 *   <ROW 0 in binary>\n
 *   ...
 *   <ROW 119 in binary>\n
 *   </FRAME>\n
 */
static void send_frame_over_uart_binary(void)
{
    static const char frame_start[] = "<FRAME>\n";
    static const char frame_end[]   = "</FRAME>\n";

    /* Helper lambda – send one character, busy-waiting until TX FIFO has
     * space.  Zephyr provides uart_poll_out() for exactly that. */

    /* 1) Start tag */
    for (size_t i = 0; i < sizeof(frame_start) - 1; i++) {
        uart_poll_out(uart_dev, frame_start[i]);
    }
    for (int y = 0; y < IMG_HEIGHT; y++) {
        for (int x = 0; x < IMG_WIDTH; x++) {
            uint8_t pixel = image[y * IMG_WIDTH + x];
            uart_poll_out(uart_dev, pixel);
        }
    }
    

    /* 3) End tag */
    for (size_t i = 0; i < sizeof(frame_end) - 1; i++) {
        uart_poll_out(uart_dev, frame_end[i]);
    }
}


void hm_i2c_write(uint16_t addr, uint8_t reg)
{
    uint8_t cfg[3] = {(uint8_t)(addr >> 8), (uint8_t)(addr & 0x00FF), reg};
    i2c_write_dt(&dev_i2c, cfg, 3);
}

void init_cam(void)
{
    //    hm_i2c_write(REG_SW_RESET, 0x00); //Software reset, reset all serial interface registers to its default values
    hm_i2c_write(0x0104, 0x00);
    hm_i2c_write( REG_MODE_SELECT, 0x00);//go to stand by mode
    hm_i2c_write( REG_ANA_REGISTER_17, 0x00);//register to change the clk source(osc:1 mclk:0), if no mclk it goes to osc by default
    //hm_i2c_write( REG_TEST_PATTERN_MODE, TEST_PATTERN);//Enable the test pattern, set it to walking 1

    hm_i2c_write(0x0601, 0x11);
    hm_i2c_write( REG_BIN_MODE, 0x00);//VERTICAL BIN MODE
    hm_i2c_write( REG_QVGA_WIN_EN, 0x01);//Set line length LSB to QQVGA => enabled: makes the image 160(row)*240(col)
//    disable: image 160*320 //In test pattern mode, enabling this does not have any effect

    /*looking at lattice cfg setting*/
    //hm_i2c_write(0x0103,0x00);

    //100*100 optimization
    hm_i2c_write( REG_BIN_RDOUT_X, 0x01);//Horizontal Binning enable
    hm_i2c_write( REG_BIN_RDOUT_Y, 0x01);//vertical Binning enable => this register should be always 0x03 because we never go more than 160 for the height
        //frame timing control
    hm_i2c_write(REG_FRAME_LENGTH_PCK_H,0x01);
    hm_i2c_write(REG_FRAME_LENGTH_PCK_L,0x78);//changed by Ali

    hm_i2c_write(REG_FRAME_LENGTH_LINES_H,0x02);//changed by Ali
    hm_i2c_write(REG_FRAME_LENGTH_LINES_L,0x12);//changed by Ali   

    /*looking at lattice cfg setting*/
    //hm_i2c_write(0x0103,0x00);


    hm_i2c_write(0x3044,0x0A);
    hm_i2c_write(0x3045,0x00);
    hm_i2c_write(0x3047,0x0A);
    hm_i2c_write(0x3050,0xC0);
    hm_i2c_write(0x3051,0x42);
//    hm_i2c_write(0x3052,0x50);
    hm_i2c_write(0x3053,0x00);
    hm_i2c_write(0x3054,0x03);
    hm_i2c_write(0x3055,0xF7);
    hm_i2c_write(0x3056,0xF8);
    hm_i2c_write(0x3057,0x29);
    hm_i2c_write(0x3058,0x1F);
//    hm_i2c_write(0x3059,0x1E);//bit control
    hm_i2c_write(0x3064,0x00);
    hm_i2c_write(0x3065,0x04);

    //black level control
    hm_i2c_write(0x1000,0x43);
    hm_i2c_write(0x1001,0x40);
    hm_i2c_write(0x1002,0x32);
    hm_i2c_write(0x1003,0x08);//default from lattice 0x08
    hm_i2c_write(0x1006,0x01);
    hm_i2c_write(0x1007,0x08);//default from lattice 0x08

    hm_i2c_write(0x0350,0x7F);
    

    //Sensor reserved
    hm_i2c_write(0x1008,0x00);
    hm_i2c_write(0x1009,0xA0);
    hm_i2c_write(0x100A,0x60);
    hm_i2c_write(0x100B,0x90);//default from lattice 0x90
    hm_i2c_write(0x100C,0x40);//default from lattice 0x40

    //Vsync, hsync and pixel shift register
//    hm_i2c_write(0x1012,0x07);//changed by Ali
    hm_i2c_write(0x1012,0x00);//lattice value

    //Statistic control and read only
    hm_i2c_write(0x2000,0x07);
    hm_i2c_write(0x2003,0x00);
    hm_i2c_write(0x2004,0x1C);
    hm_i2c_write(0x2007,0x00);
    hm_i2c_write(0x2008,0x58);
    hm_i2c_write(0x200B,0x00);
    hm_i2c_write(0x200C,0x7A);
    hm_i2c_write(0x200F,0x00);
    hm_i2c_write(0x2010,0xB8);
    hm_i2c_write(0x2013,0x00);
    hm_i2c_write(0x2014,0x58);
    hm_i2c_write(0x2017,0x00);
    hm_i2c_write(0x2018,0x9B);

    //Automatic exposure gain control
    hm_i2c_write(0x2100,0x01);
    hm_i2c_write(0x2101,0x70);//0x70);//lattice 0xA0
    hm_i2c_write(0x2102,0x01);//lattice 0x06
    hm_i2c_write(0x2104,0x07);
    hm_i2c_write(0x2105,0x03);
    hm_i2c_write(0x2106,0xA4);
    hm_i2c_write(0x2108,0x33);
    hm_i2c_write(0x210A,0x00);
    //hm_i2c_write(0x210C,0x04);
    hm_i2c_write(0x210B,0x80);
    hm_i2c_write(0x210F,0x00);
    hm_i2c_write(0x2110,0xE9);
    hm_i2c_write(0x2111,0x01);
    hm_i2c_write(0x2112,0x17);
    hm_i2c_write(0x2150,0x03);

    //Sensor exposure gain
    hm_i2c_write(0x0205,0x05);//Vikram
    hm_i2c_write(0x020E,0x01);//Vikram
    hm_i2c_write(0x020F,0x00);//Vikram
    hm_i2c_write(0x0202,0x01);//Vikram
    hm_i2c_write(0x0203,0x08);//Vikram

    hm_i2c_write( REG_OSC_CLK_DIV, 0x00);//This is effective when we use external clk, Use the camera in the gated clock mode to make the clock zero when there is no data

    hm_i2c_write( REG_BIT_CONTROL, 0x20);//Set the output to send 1 bit serial

    hm_i2c_write( REG_PMU_PROGRAMMABLE_FRAMECNT, 0x01);//set the number of frames to be sent out, it sends N frames

    hm_i2c_write(0x0104, 0x00);
    hm_i2c_write(0x0100 ,0x01);
}


void capture_and_send_frame(const struct device *gpio0_dev)
{
    capture_frame(gpio0_dev);
    send_frame_over_uart_binary();
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

    init_cam();

    
    
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
