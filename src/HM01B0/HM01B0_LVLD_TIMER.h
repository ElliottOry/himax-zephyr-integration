/*
 * HM01B0_LVLD_TIMER.h
 *
 *  Created on: April 7, 2019
 *      Author: Ali
 */

#ifndef HM01B0_LVLD_TIMER_H_
#define HM01B0_LVLD_TIMER_H_

#include <stdbool.h>
#include <stdint.h>
#include <nrfx_timer.h>
#include <nrfx_gpiote.h>
#include "HM01B0_def_values.h"

void lvld_timer_enable(void);
void lvld_timer_disable(void);
void lvld_timer_init(void);

#endif /* HM01B0_LVLD_TIMER_H_ */