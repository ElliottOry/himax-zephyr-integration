# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(custom_target_3)

target_sources(app PRIVATE 
src/main.c
src/ble_manager.c
src/HM01B0/HM01B0_CLK.c)
