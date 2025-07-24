/*
 * HM01B0_GPIO.h
 *
 *  Created on: Nov 20, 2018
 *      Author: Ali Najafi
 */

#ifndef HM01B0_GPIO_H_
#define HM01B0_GPIO_H_

/*  Standard C Included Files */
#include <stdbool.h>
#include <nrfx_gpiote.h>

#include "HM01B0_def_values.h"
#include "HM01B0_LVLD_TIMER.h"

void gpio_setting_init(void);
void gpio_setting_uninit(void);

/*!
* @brief Call back for PINT Pin interrupt 0-7.
*/
void in_pin_handler_CAM_LINE_VALID(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
void in_pin_handler_CAM_FRAME_VALID(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action);

#endif /* HM01B0_GPIO_H_ */