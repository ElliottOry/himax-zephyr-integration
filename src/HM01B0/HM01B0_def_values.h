#ifndef HM01B0DEFVALUES_H_
#define HM01B0DEFVALUES_H_

#define PIN_OUT       28            /* any free GPIO          */
#define GPIOTE_CH     0
#define PPI_CH        0

#define IMG_WIDTH 162
#define IMG_HEIGHT 119
#define IMAGE_SIZE (IMG_WIDTH * IMG_HEIGHT)


#define PIN_MCLK 3
#define MCLK_TIMER_INSTANCE 0

/* Pin mapping – LVLD (HSYNC) is used as SPIS-CSN as well                    */
#define PIN_VSYNC        27          /* FVLD from HM01B0                     */
#define PIN_LVLD_CSN     11          /* LVLD → CSN to SPIS0                  */
#define PIN_PCLK_SCK      8          /* PCLK → SCK to SPIS0                  */
#define PIN_D0_MOSI       6          /* D0   → MOSI to SPIS0                 */

#define CAM_FRAME_VALID                   PIN_VSYNC
#define CAM_LINE_VALID                    PIN_LVLD_CSN
//#define CAM_INT                           NRF_GPIO_PIN_MAP(0, 9)   // Input
#define CAM_MCLK_IN_FROM_MCU              PIN_MCLK


#define CAM_D0          PIN_D0_MOSI

#define CAM_SPI_CS_OUT   PIN_LVLD_CSN

#define PIN_GATE          PIN_LVLD_CSN         /* start capture when high */
#define GPIOTE_CH_GATE   1
#define PPI_CH_GATE      1

#define PIN_PCLK          PIN_PCLK_SCK
#define GPIOTE_CH_PCLK    3
#define PPI_CH_PCLK       3

/* HSYNC → timer STOP bridge */
#define PIN_HSYNC         7
#define GPIOTE_CH_HSYNC   2              /* free GPIOTE channel            */
#define PPI_CH_REL        2              /* PPI channel for TASKS_STOP      */

/* Peripherals from DTS */
#define UART_NODE       DT_NODELABEL(uart0)
#define I2C_NODE        DT_ALIAS(hmm)
#define SPI_NODE        DT_NODELABEL(gendev)

/* nRF52832 RAM span that is reachable by EasyDMA (64 kB) */
#define DMA_RAM_START   0x20000000u
#define DMA_RAM_END     0x20010000u

static atomic_t line_idx;
static atomic_t capturing;

#endif /* HM01B0DEFVALUES_H_ */