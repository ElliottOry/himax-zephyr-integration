/*
 * HM01B0_CLK.h
 *
 *  Created on: Nov 25, 2018
 *      Author: Ali Najafi
 */

#ifndef HM01B0_CLK_H_
#define HM01B0_CLK_H_

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <nrfx_timer.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include "HM01B0_def_values.h"



/** Camera timer instance (NRFX timer instance) */
extern const nrfx_timer_t cam_timer;

#define CAM_TIMER (&cam_timer)

void hm_clk_out(void);
void hm_clk_enable(bool enable);

#endif /* HM01B0_CLK_H_ */
