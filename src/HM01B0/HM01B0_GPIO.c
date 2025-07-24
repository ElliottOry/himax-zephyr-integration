/*
 * HM01B0_GPIO.c
 *
 *  Created on: Nov 20, 2018
 *      Author: Ali Najafi
 */
#include "HM01B0_GPIO.h"

void in_pin_handler_CAM_LINE_VALID(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  if ((unsigned)atomic_get(&line_idx) < IMG_HEIGHT)
  {
    // here we need to activate SPI CS; enable the lvld_timer; and activate the CAM_LINE_VALID interrupt; increase the counter of lines
    lvld_timer_enable();
    NRF_GPIO->OUTCLR = 1UL << CAM_SPI_CS_OUT;
    atomic_inc(&line_idx);
  }
  else
  {
    nrfx_gpiote_trigger_enable(0, CAM_LINE_VALID, true);
    nrfx_gpiote_trigger_enable(0, CAM_FRAME_VALID, true);
    lvld_timer_disable();
    NRF_GPIO->OUTSET = 1UL << CAM_SPI_CS_OUT;
    NRF_LOG_RAW_INFO("%d lvld pin done\n", systemTimeGetMs());
    hm_set_capture_done();
  }
}

void in_pin_handler_CAM_FRAME_VALID(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  static bool start = true;

  if (gpioRead(CAM_FRAME_VALID) && start) {
    // nrf_gpio_pin_clear(CAM_SPI_CS_OUT);
    lvld_timer_enable();
    start = false;
    nrf_drv_gpiote_in_event_disable(CAM_LINE_VALID);
    NRF_LOG_RAW_INFO("%08d image start\n", systemTimeGetMs());
  } else if (gpioRead(CAM_FRAME_VALID) == 0 && !start) {
    NRF_GPIO->OUTSET = 1UL << CAM_SPI_CS_OUT;
    nrf_drv_gpiote_in_event_disable(CAM_FRAME_VALID);
    start = true;
    NRF_LOG_RAW_INFO("%08d image end\n", systemTimeGetMs());
  }

  // if (line_count > 0) {

  // } else {
  //   nrf_gpio_pin_clear(CAM_SPI_CS_OUT);
  //   // nrf_drv_gpiote_in_event_disable(CAM_FRAME_VALID);
  //   line_count = 0;
  //   lvld_timer_enable();
  // }

  // NRF_LOG_RAW_INFO("%d frame valid\n", systemTimeGetMs());
}

void gpio_setting_uninit(void)
{
  nrf_drv_gpiote_in_uninit(CAM_FRAME_VALID);
  nrf_drv_gpiote_in_uninit(CAM_LINE_VALID);
}

void gpio_setting_init(void)
{
  ret_code_t err_code;

  /*toggling*/
  nrfx_gpiote_input_config_t in_config_frmvld = NRFX_GPIOTE_DEFAULT_PULL_CONFIG;
  in_config_frmvld.pull = NRF_GPIO_PIN_NOPULL;

  err_code = nrf_drv_gpiote_in_init(CAM_FRAME_VALID, &in_config_frmvld, in_pin_handler_CAM_FRAME_VALID);
  APP_ERROR_CHECK(err_code);

  /*Finds falling for line valid edge instead of just toggling*/
  nrf_drv_gpiote_in_config_t in_config_lnvld = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
  in_config_lnvld.pull = NRF_GPIO_PIN_NOPULL;

  err_code = nrf_drv_gpiote_in_init(CAM_LINE_VALID, &in_config_lnvld, in_pin_handler_CAM_LINE_VALID);
  APP_ERROR_CHECK(err_code);
}
