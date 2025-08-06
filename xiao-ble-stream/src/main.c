/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Author: tekritesh@github.com
 */

#include <stddef.h>
#include <stdint.h>
#include <zephyr/sys/printk.h>
// #include <ble_manager.h>

extern void run_peripheral_step( uint16_t seconds);
 
int main(void)
{
	run_peripheral_step(0);
	return 0;
}