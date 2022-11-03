# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.13)

option(FLOW_CONTROL "Enable Hardware Flow-control on the UARTs" FALSE)

include(pico-sdk/pico_sdk_init.cmake)

project(pico_uart_bridge)

pico_sdk_init()
  
add_executable(uart_bridge uart-bridge.c usb-descriptors.c)

target_include_directories(uart_bridge PUBLIC
	./
	pico-sdk/lib/tinyusb/src)

target_link_libraries(uart_bridge
	hardware_flash
	pico_multicore
	pico_stdlib
	tinyusb_device)

if(FLOW_CONTROL)
target_compile_definitions(uart_bridge PUBLIC FLOW_CONTROL=1)
endif()

if(LINE_CONTROL)
target_compile_definitions(uart_bridge PUBLIC LINE_CONTROL=1)
endif()

pico_add_extra_outputs(uart_bridge)
