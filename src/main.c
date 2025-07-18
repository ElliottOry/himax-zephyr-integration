/*
* HM01B0 capture example – interrupt driven, DMA-ready version
*
* Copyright (c) 2024 Open Pixel Systems
* SPDX-License-Identifier: Apache-2.0
*/

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/atomic.h>
#include <hal/nrf_power.h>
#include <hal/nrf_clock.h>

#include <hal/nrf_spis.h>
#include <nrfx_spis.h>
//#include <drivers/spi/spi_nrfx_spis.h>
#include <nrfx_gpiote.h>
#include <nrfx_ppi.h>
#include <helpers/nrfx_gppi.h>

#define PIN_OUT       28            /* any free GPIO          */
#define GPIOTE_CH     0
#define PPI_CH        0
#include "HM01B0Regs.h"

/* -------------------------------------------------------------------------- */
/* Compile-time configuration                                                 */
/* -------------------------------------------------------------------------- */
#define IMG_WIDTH        160
#define IMG_HEIGHT       120
#define IMAGE_SIZE      (IMG_WIDTH * IMG_HEIGHT)

/* Pin mapping – LVLD (HSYNC) is used as SPIS-CSN as well                    */
#define PIN_VSYNC        27          /* FVLD from HM01B0                     */
#define PIN_LVLD_CSN     11          /* LVLD → CSN to SPIS0                  */
#define PIN_PCLK_SCK      8          /* PCLK → SCK to SPIS0                  */
#define PIN_D0_MOSI       6          /* D0   → MOSI to SPIS0                 */


#define PIN_GATE          PIN_LVLD_CSN         /* goes high when we may ACQUIRE */
#define GPIOTE_CH_GATE   1
#define PPI_CH_GATE      1

/* HSYNC → SPIS RELEASE bridge */
#define PIN_HSYNC         7
#define GPIOTE_CH_HSYNC   2              /* free GPIOTE channel            */
#define PPI_CH_REL        2              /* PPI channel for TASKS_RELEASE   */


/* Peripherals from DTS */
#define UART_NODE       DT_NODELABEL(uart0)
#define I2C_NODE        DT_ALIAS(hmm)
#define SPI_NODE        DT_NODELABEL(gendev)

/* SPIS slave, 8‑bit, MSB‑first, Mode 0 */
#define SPI_OP (SPI_WORD_SET(8) | SPI_OP_MODE_SLAVE | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA)

/* nRF52832 RAM span that is reachable by EasyDMA (64 kB) */
#define DMA_RAM_START   0x20000000u
#define DMA_RAM_END     0x20010000u

/* -------------------------------------------------------------------------- */
/* Global objects                                                             */
/* -------------------------------------------------------------------------- */
__aligned(4)  /* EasyDMA friendly                                             */
static uint8_t image[IMAGE_SIZE];

/*  Compile-time test: linker may NOT move the buffer out of DMA RAM         */

static inline void hm_i2c_write(uint16_t reg, uint8_t val);


static struct spi_dt_spec spispec = SPI_DT_SPEC_GET(SPI_NODE, SPI_OP, 0);


static const struct device *uart_dev   = DEVICE_DT_GET(UART_NODE);
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);
static const struct device *gpio0_dev  = DEVICE_DT_GET(DT_NODELABEL(gpio0));


/* -------------------------------------------------------------------------- */
/* Synchronisation primitives                                                 */
/* -------------------------------------------------------------------------- */
static struct k_sem  frame_sem;          /* “frame is ready” semaphore       */
static struct k_sem  line_sem;           /* given by SPIS END callback       */

/* A dedicated work-queue thread is faster than the system work-queue         */
#define LINE_THREAD_STACK_SZ 768
K_THREAD_STACK_DEFINE(line_stack, LINE_THREAD_STACK_SZ);
static struct k_thread line_thread_data;

static atomic_t line_idx;
static atomic_t capturing;
static uint8_t *current_dst;

/* Forward declaration */
static void arm_next_spis_transfer(void);


/* -------------------------------------------------------------------------- */
/* Worker thread – captures one line per iteration                            */
/* -------------------------------------------------------------------------- */
static void line_thread(void *p1, void *p2, void *p3)
{
ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

while (true) {
k_sem_take(&line_sem, K_FOREVER);     /* unblocked by SPIS END ISR   */

if (!atomic_get(&capturing)) {
continue;                         /* ignore stray events         */
}

/* Capture one line: wait for VSYNC event or previous DMA completion */
arm_next_spis_transfer();

/* Advance line index and write pointer for next line */
atomic_inc(&line_idx);
current_dst += IMG_WIDTH;

/* If we've captured the last line, end frame */
if (atomic_get(&line_idx) >= IMG_HEIGHT) {
    printk("Lineeee %3u", (unsigned)atomic_get(&line_idx) - 1);
atomic_clear(&capturing);
k_sem_give(&frame_sem);
}
}
}

/* Arms SPIS for the next 240-byte DMA reception                              */
static void arm_next_spis_transfer(void)
{
struct spi_buf rx  = { .buf = current_dst, .len = IMG_WIDTH};
struct spi_buf_set rxset = { .buffers = &rx, .count = 1 };

//gpio_pin_configure(gpio0_dev, 28, GPIO_OUTPUT_ACTIVE);
spi_read_dt(&spispec, &rxset);
//gpio_pin_configure(gpio0_dev, 28, GPIO_OUTPUT_INACTIVE);
/*
uint32_t got = rx.len;
if (got != 160) {               
    printk("Line %3u: got %3lu bytes\n", (unsigned)atomic_get(&line_idx) - 1, (unsigned long)got);
}
    */
k_sem_give(&line_sem);
}

/* -------------------------------------------------------------------------- */
/* VSYNC ISR – starts a new frame                                             */
/* -------------------------------------------------------------------------- */
static struct gpio_callback vsync_cb;

static void vsync_isr(const struct device *dev,
    struct gpio_callback *cb,
    uint32_t pins)
    {
    if (gpio_pin_get(dev, PIN_VSYNC)) {          /* rising edge = frame start */
    atomic_set(&capturing, 1);
    atomic_set(&line_idx, 0);
    current_dst = image;

    k_sem_reset(&line_sem);                  /* flush stale grants        */
    /* Wake the line-capture thread so that it can start the very first
            * DMA transaction from thread context. Doing it here (ISR) would
            * violate Zephyr's rules because spi_read_dt() might sleep.        */
    k_sem_give(&line_sem);
    }
}

/* -------------------------------------------------------------------------- */
/* HM01B0 helper (unchanged)                                                   */
/* -------------------------------------------------------------------------- */
static inline void hm_i2c_write(uint16_t reg, uint8_t val)
{
uint8_t buf[] = { reg >> 8, reg & 0xff, val };
i2c_write_dt(&dev_i2c, buf, sizeof buf);
}

/* … init_cam() is identical to your original code …                         */

/* -------------------------------------------------------------------------- */
/* UART helper (unchanged)                                                    */
/* -------------------------------------------------------------------------- */
static void send_frame_over_uart_binary(void)
{
static const char hdr[]  = "<FRAME>\n";
static const char tail[] = "</FRAME>\n";

for (int i = 0; i < sizeof hdr - 1; i++)  uart_poll_out(uart_dev, hdr[i]);
for (size_t i = 0; i < IMAGE_SIZE; i++)   uart_poll_out(uart_dev, image[i]);
for (int i = 0; i < sizeof tail - 1; i++) uart_poll_out(uart_dev, tail[i]);
}

/* -------------------------------------------------------------------------- */
/* main                                                                       */
/* -------------------------------------------------------------------------- */
void init_cam(void)
{



    hm_i2c_write( REG_MODE_SELECT, 0x00);//go to stand by mode
    hm_i2c_write( REG_ANA_REGISTER_17, 0x00);//register to change the clk source(osc:1 mclk:0), if no mclk it goes to osc by default
    hm_i2c_write( REG_TEST_PATTERN_MODE, 0x00);//Enable the test pattern, set it to walking 1

    
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

    //frame timing control
//    hm_i2c_write(0x0340,0x02);//changed by Ali
//    hm_i2c_write(0x0341,0x32);//changed by Ali    
////    hm_i2c_write(0x0340,0x0C);
////    hm_i2c_write(0x0341,0x5C);
//
//    hm_i2c_write(0x0342,0x01);
//    hm_i2c_write(0x0343,0x78);//changed by Ali
//    hm_i2c_write(0x0343,0x78);

//    hm_i2c_write(0x3010,0x01); //done in lower lines
//    hm_i2c_write(0x0383,0x00); //done in lower lines
//    hm_i2c_write(0x0387,0x00); //done in lower lines
//    hm_i2c_write(0x0390,0x00); //done in lower lines
//    hm_i2c_write(0x3059,0x42); //done in lower lines
//    hm_i2c_write(0x3060,0x51); //done in lower lines
        hm_i2c_write( REG_OSC_CLK_DIV, 0x30);//This is effective when we use external clk, Use the camera in the gated clock mode to make the clock zero when there is no data

        hm_i2c_write( REG_BIT_CONTROL, 0x20);//Set the output to send 1 bit serial
        //hm_i2c_write(0x3062, 0xF1);
        hm_i2c_write(0x3060, 0x30);
        hm_i2c_write(0x3023, 0x05);

        hm_i2c_write( REG_PMU_PROGRAMMABLE_FRAMECNT, 0x01);//set the number of frames to be sent out, it sends N frames
}

/* -------------------------------------------------------------------------- */
/* Scope‑pin helper – toggles P0.17 on every SPIS1 EVENTS_END via PPI         */
/* -------------------------------------------------------------------------- */
static void scope_pin_init(void)
{
    /* 1) P0.17 as output driven by GPIOTE task                                */
    NRF_GPIO->DIRSET = 1UL << PIN_OUT;
    NRF_GPIOTE->CONFIG[GPIOTE_CH] = (GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos) |
                                    (PIN_OUT                 << GPIOTE_CONFIG_PSEL_Pos) |
                                    (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos);

    /* 2) Route SPIS1 EVENTS_END to that task via PPI channel 0                */
    NRF_PPI->CH[PPI_CH].EEP = (uint32_t)&NRF_SPIS1->EVENTS_ACQUIRED;
    NRF_PPI->CH[PPI_CH].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[GPIOTE_CH];

    /* 3) Enable the PPI channel                                              */
    NRF_PPI->CHENSET = 1UL << PPI_CH;
}


void gate_trigger_init(void)
{
        NRF_GPIO->PIN_CNF[PIN_GATE] =
            (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos)  |
            (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)|
            (GPIO_PIN_CNF_PULL_Pullup  << GPIO_PIN_CNF_PULL_Pos);

        NRF_GPIOTE->CONFIG[GPIOTE_CH_GATE] =
            (GPIOTE_CONFIG_MODE_Event     << GPIOTE_CONFIG_MODE_Pos) |
            (PIN_GATE                     << GPIOTE_CONFIG_PSEL_Pos) |
            (GPIOTE_CONFIG_POLARITY_HiToLo<< GPIOTE_CONFIG_POLARITY_Pos);

        /* 2) Route the event to TASKS_ACQUIRE */
        NRF_PPI->CH[PPI_CH_GATE].EEP = (uint32_t)&NRF_GPIOTE->EVENTS_IN[GPIOTE_CH_GATE];
        NRF_PPI->CH[PPI_CH_GATE].TEP = (uint32_t)&NRF_SPIS1->TASKS_ACQUIRE;

        /* 3) Enable the PPI channel */
        NRF_PPI->CHENSET = 1UL << PPI_CH_GATE;
}

void gate_trigger_init(void)
{
        NRF_GPIO->PIN_CNF[PIN_GATE] =
            (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos)  |
            (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)|
            (GPIO_PIN_CNF_PULL_Pullup  << GPIO_PIN_CNF_PULL_Pos);

        NRF_GPIOTE->CONFIG[GPIOTE_CH_GATE] =
            (GPIOTE_CONFIG_MODE_Event     << GPIOTE_CONFIG_MODE_Pos) |
            (PIN_GATE                     << GPIOTE_CONFIG_PSEL_Pos) |
            (GPIOTE_CONFIG_POLARITY_HiToLo<< GPIOTE_CONFIG_POLARITY_Pos);

        /* 2) Route the event to TASKS_ACQUIRE */
        NRF_PPI->CH[PPI_CH_GATE].EEP = (uint32_t)&NRF_GPIOTE->EVENTS_IN[GPIOTE_CH_GATE];
        NRF_PPI->CH[PPI_CH_GATE].TEP = (uint32_t)&NRF_SPIS1->TASKS_ACQUIRE;

        /* 3) Enable the PPI channel */
        NRF_PPI->CHENSET = 1UL << PPI_CH_GATE;
}


/* -------------------------------------------------------------------------- */
/* Release helper – SPIS1 RELEASE fires when HSYNC (LVLD) falls               */
/* -------------------------------------------------------------------------- */
static void release_trigger_init(void)
{
    /* 1) Configure PIN_HSYNC as input (it already is) and generate an event on Hi→Lo */
    NRF_GPIO->PIN_CNF[PIN_HSYNC] =
          (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos)  |
          (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)|
          (GPIO_PIN_CNF_PULL_Pulldown  << GPIO_PIN_CNF_PULL_Pos);
    NRF_GPIOTE->CONFIG[GPIOTE_CH_HSYNC] =
          (GPIOTE_CONFIG_MODE_Event      << GPIOTE_CONFIG_MODE_Pos) |
          (PIN_HSYNC                      << GPIOTE_CONFIG_PSEL_Pos) |
          (GPIOTE_CONFIG_POLARITY_LoToHi  << GPIOTE_CONFIG_POLARITY_Pos);

    /* 2) Route the event to TASKS_RELEASE */
    NRF_PPI->CH[PPI_CH_REL].EEP = (uint32_t)&NRF_GPIOTE->EVENTS_IN[GPIOTE_CH_HSYNC];
    NRF_PPI->CH[PPI_CH_REL].TEP = (uint32_t)&NRF_SPIS1->TASKS_RELEASE;

    /* 3) Enable PPI channel */
    NRF_PPI->CHENSET = 1UL << PPI_CH_REL;
}



int main(void)
{
//spispec.config = spi_cfg_pwr1;
__ASSERT(((uintptr_t)image) >= DMA_RAM_START &&((uintptr_t)image + IMAGE_SIZE) <= DMA_RAM_END, "image[] must live in the first 64 kB of SRAM (EasyDMA)");
if (!device_is_ready(uart_dev)      ||
    !device_is_ready(dev_i2c.bus)   ||
    !device_is_ready(spispec.bus)   ||
    !device_is_ready(gpio0_dev)) {
    printk("Device not ready\n");
    return -ENODEV;
}
//printk("poop");

/* Keep the system in constant‑latency mode to avoid first‑byte corruption */
NRF_POWER->TASKS_CONSTLAT = 1;

/* Start the high‑frequency crystal so SPIS always samples with HFCLK */
NRF_CLOCK->TASKS_HFCLKSTART = 1;
while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {
}
NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;

/* Enable END→ACQUIRE shortcut once the SPIS driver has been initialised */
/* Enable END→ACQUIRE shortcut once the SPIS driver has been initialised */
{
    NRF_SPIS_Type *spis =
        (NRF_SPIS_Type *)DT_REG_ADDR(DT_NODELABEL(spi1));  /* SPIS1 base */
    //spis->SHORTS |= SPIS_SHORTS_END_ACQUIRE_Msk;
}
scope_pin_init();
gate_trigger_init();
release_trigger_init();
/* Create semaphores after driver init so ISR may use them immediately    */
k_sem_init(&frame_sem, 0, 1);
k_sem_init(&line_sem,  0, 1);


/* Spawn dedicated line-capture thread                                    */
k_thread_create(&line_thread_data, line_stack, LINE_THREAD_STACK_SZ,
                line_thread, NULL, NULL, NULL,
                K_PRIO_PREEMPT(1), 0, K_NO_WAIT);

init_cam();

/* --- VSYNC pin -------------------------------------------------------- */
gpio_pin_configure(gpio0_dev, PIN_VSYNC, GPIO_INPUT | GPIO_PULL_DOWN);
gpio_init_callback(&vsync_cb, vsync_isr, BIT(PIN_VSYNC));
gpio_add_callback(gpio0_dev, &vsync_cb);
gpio_pin_interrupt_configure(gpio0_dev, PIN_VSYNC, GPIO_INT_EDGE_RISING);
/* --- CSN pin -------------------------------------------------------- */
        uint8_t addr_readback[2];
        uint8_t val;

        /* BIT_CONTROL should read back 0x20 for 1-bit serial mode. */
        addr_readback[0] = 0x03; addr_readback[1] = 0x50;
        if (i2c_write_read_dt(&dev_i2c, addr_readback, 2, &val, 1) == 0)
            printk("QVGA win (0x0350) = 0x%02X\n", val);

        /* OSC_CLK_DIV should read back 0x00 (MSB-first, ÷8, non-gated). */
        addr_readback[0] = 0x03; addr_readback[1] = 0x81;
        if (i2c_write_read_dt(&dev_i2c, addr_readback, 2, &val, 1) == 0)
            printk("BIN_RDOUT_Y (0x0381) = 0x%02X\n", val);

        /* TEST_PATTERN_MODE – should be 0x00 (disabled) */
        addr_readback[0] = 0x10; addr_readback[1] = 0x06;
        if (i2c_write_read_dt(&dev_i2c, addr_readback, 2, &val, 1) == 0)
            printk("IMG_MODE_SEL (0x1006) = 0x%02X\n", val);
/* ------------------------- main loop ---------------------------------- */

while (true) {
hm_i2c_write(REG_MODE_SELECT, 0x03);   /* start streaming  */
k_sem_take(&frame_sem, K_FOREVER);     /* wait for 1 frame */
hm_i2c_write(REG_MODE_SELECT, 0x00);   /* standby          */

send_frame_over_uart_binary();
}
}