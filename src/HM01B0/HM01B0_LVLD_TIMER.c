/*
 * HM01B0_LVLD_TIMER.h
 *
 *  Created on: April 25, 2019
 *      Author: Ali Najafi
 */

#include "HM01B0_LVLD_TIMER.h"

// #define LVLD_TIMER_VALUE 2800 // the second number in multiplication is equal to 64/cam_mclk_freq; if cam_mcl_freq=8MHz => 8

#define CAM_MCLK_FREQ_MHZ 8
#ifdef QQVGA
  #define LVLD_TIMER_VALUE 2200 // the second number in multiplication is equal to 64/cam_mclk_freq; if cam_mcl_freq=8MHz => 8
#else
  #define LVLD_TIMER_VALUE ((IMG_WIDTH + 20) * (64 / CAM_MCLK_FREQ_MHZ) * 2) // the second number in multiplication is equal to 64/cam_mclk_freq; if cam_mcl_freq=8MHz => 8
#endif

#define TIMER_LVLD NRF_TIMER_INST_GET(4)

void lvld_timer_enable(void)
{
  nrfx_timer_enable(TIMER_LVLD);
}

void lvld_timer_disable(void)
{
  nrfx_timer_disable(TIMER_LVLD); 
}

/**
 * @brief Handler for timer events.
 */
void timer_lvld_event_handler(nrf_timer_event_t event_type, void* p_context)
{
  /*TODO: REVIEW THIS IN BENJI CODE*/
  nrfx_gpiote_out_clear(0, CAM_SPI_CS_OUT);
  lvld_timer_disable();
  // if (line_count < IMAGE_HEIGHT)
  // {
  //   lvld_timer_disable();
  //   NRF_GPIO->OUTSET = 1UL << CAM_SPI_CS_OUT;
  //   spiSlaveSetBuffersBackWithLineCount(line_count);
  //   nrf_drv_gpiote_in_event_enable(CAM_LINE_VALID, true);
  // }
  // else
  // {
  //   nrf_drv_gpiote_in_event_disable(CAM_LINE_VALID);
  //   nrf_drv_gpiote_in_event_disable(CAM_FRAME_VALID);
  //   lvld_timer_disable();
  //   NRF_GPIO->OUTSET = 1UL << CAM_SPI_CS_OUT;
  //   NRF_LOG_RAW_INFO("%d lvld event done\n", systemTimeGetMs());
  //   hm_set_capture_done();
  // }
}

void lvld_timer_init(void)
{
  uint32_t err_code = NRFX_SUCCESS;
  //Configure TIMER_LED for generating simple light effect - leds on board will invert his state one after the other.
  nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG(8000000);
  err_code = nrfx_timer_init(TIMER_LVLD, &timer_cfg, timer_lvld_event_handler);
  //APP_ERROR_CHECK(err_code);

  nrfx_timer_extended_compare(
      TIMER_LVLD, NRF_TIMER_CC_CHANNEL4, LVLD_TIMER_VALUE, NRF_TIMER_SHORT_COMPARE4_CLEAR_MASK, true);
}

/** @} */
