# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.13)

include(pico-sdk/pico_sdk_init.cmake)

project(pico_uart_bridge)

if (NOT PICO_BOARD)
	set(PICO_BOARD pico CACHE STRING "Target board")
endif()

pico_sdk_init()

add_executable(uart_bridge uart-bridge.c usb-descriptors.c)

target_include_directories(uart_bridge PUBLIC
	./
	pico-sdk/lib/tinyusb/src)

foreach(pin UART0_TX UART0_RX UART1_TX UART1_RX)
	set(BRIDGE_${pin}_PIN "" CACHE STRING "Override bridge ${pin} GPIO")
	if (NOT "${BRIDGE_${pin}_PIN}" STREQUAL "")
		target_compile_definitions(uart_bridge PRIVATE
			BRIDGE_${pin}_PIN=${BRIDGE_${pin}_PIN})
	endif()
endforeach()

target_link_libraries(uart_bridge
	hardware_flash
	pico_multicore
	pico_status_led
	pico_stdlib
	tinyusb_device)

pico_add_extra_outputs(uart_bridge)
