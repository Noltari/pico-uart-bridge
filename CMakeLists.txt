# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.13)

include(pico-sdk/pico_sdk_init.cmake)

project(pico_uart_bridge)

pico_sdk_init()
  
add_executable(uart_bridge uart-bridge.c usb-descriptors.c)

target_include_directories(uart_bridge PUBLIC
	pico-sdk/lib/tinyusb/src
	pico-sdk/src/rp2_common/pico_stdio_usb/include)

target_link_libraries(uart_bridge
	pico_multicore
	pico_stdlib
	tinyusb_device)

pico_add_extra_outputs(uart_bridge)
