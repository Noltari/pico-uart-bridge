// SPDX-License-Identifier: MIT
/*
 * Copyright 2021 Álvaro Fernández Rojas <noltari@gmail.com>
 */

#include <hardware/irq.h>
#include <hardware/structs/sio.h>
#include <hardware/uart.h>
#include <pico/multicore.h>
#include <pico/stdio_usb.h>
#include <pico/stdlib.h>
#include <tusb.h>
#include <string.h>

#if !defined(MIN)
#define MIN(a, b) ((a > b) ? b : a)
#endif /* MIN */

#define LED_PIN 25

#define UART_ID uart0
#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define BUFFER_SIZE	64

#define DEF_BIT_RATE 115200
#define DEF_STOP_BITS 1
#define DEF_PARITY 0
#define DEF_DATA_BITS 8

static cdc_line_coding_t CDC_LC = {
	.bit_rate = DEF_STOP_BITS,
	.stop_bits = DEF_STOP_BITS,
	.parity = DEF_PARITY,
	.data_bits = DEF_DATA_BITS,
};

static uint8_t UART_BUFFER[BUFFER_SIZE];
static uint32_t UART_POS = 0;
static mutex_t UART_MTX;

static uint8_t USB_BUFFER[BUFFER_SIZE];
static uint32_t USB_POS = 0;
static mutex_t USB_MTX;

static inline uart_parity_t databits_usb2uart(uint8_t data_bits)
{
	switch (data_bits) {
		case 5:
			return 5;
		case 6:
			return 6;
		case 7:
			return 7;
		default:
			return 8;
	}
}

static inline uart_parity_t parity_usb2uart(uint8_t usb_parity)
{
	switch (usb_parity) {
		case 1:
			return UART_PARITY_ODD;
		case 2:
			return UART_PARITY_EVEN;
		default:
			return UART_PARITY_NONE;
	}
}

static inline uart_parity_t stopbits_usb2uart(uint8_t stop_bits)
{
	switch (stop_bits) {
		case 2:
			return 2;
		default:
			return 1;
	}
}

int update_uart_cfg(void)
{
	static cdc_line_coding_t last_cdc_lc = {
		.bit_rate = DEF_STOP_BITS,
		.stop_bits = DEF_STOP_BITS,
		.parity = DEF_PARITY,
		.data_bits = DEF_DATA_BITS,
	};
	int updated = 0;

	if (last_cdc_lc.bit_rate != CDC_LC.bit_rate) {
		uart_set_baudrate(UART_ID, CDC_LC.bit_rate);
		updated = 1;
	}

	if ((last_cdc_lc.stop_bits != CDC_LC.stop_bits) ||
		(last_cdc_lc.parity != CDC_LC.parity) ||
		(last_cdc_lc.data_bits != CDC_LC.data_bits)) {
		uart_set_format(UART_ID, databits_usb2uart(CDC_LC.data_bits),
						stopbits_usb2uart(CDC_LC.stop_bits),
						parity_usb2uart(CDC_LC.parity));
		updated = 1;
	}

	if (updated)
		memcpy(&last_cdc_lc, &CDC_LC, sizeof(cdc_line_coding_t));

	return updated;
}

void core1_entry(void)
{
	tusb_init();

	while (1) {
		tud_task();

		if (tud_cdc_connected()) {
			uint32_t count;

			tud_cdc_get_line_coding(&CDC_LC);

			/* Read bytes from USB */
			if (tud_cdc_available()) {
				uint32_t len;

				mutex_enter_blocking(&USB_MTX);

				len = MIN(tud_cdc_available(), BUFFER_SIZE - USB_POS);
				if (len) {
					count = tud_cdc_read(USB_BUFFER, len);
					USB_POS += count;
				}

				mutex_exit(&USB_MTX);
			}

			/* Write bytes to USB */
			if (UART_POS) {
				mutex_enter_blocking(&UART_MTX);

				count = tud_cdc_write(UART_BUFFER, UART_POS);
				if (count) {
					UART_POS -= count;
					tud_cdc_write_flush();
				}

				mutex_exit(&UART_MTX);
			}

			gpio_put(LED_PIN, 1);
		} else {
			gpio_put(LED_PIN, 0);
		}
	};
}

int main(void)
{
	uint8_t ch;
	int rc;

	mutex_init(&UART_MTX);
	mutex_init(&USB_MTX);

	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);

	uart_init(UART_ID, CDC_LC.bit_rate);

	gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
	gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

	uart_set_hw_flow(UART_ID, false, false);
	uart_set_format(UART_ID, databits_usb2uart(CDC_LC.data_bits),
		stopbits_usb2uart(CDC_LC.stop_bits),
		parity_usb2uart(CDC_LC.parity));

	multicore_launch_core1(core1_entry);

	while (1) {
		update_uart_cfg();

		/* Read bytes from UART */
		if (uart_is_readable(UART_ID)) {
			mutex_enter_blocking(&UART_MTX);

			while (uart_is_readable(UART_ID) && UART_POS < BUFFER_SIZE) {
				UART_BUFFER[UART_POS] = uart_getc(UART_ID);
				UART_POS++;
			}

			mutex_exit(&UART_MTX);
		}

		/* Write bytes to UART */
		if (USB_POS) {
			mutex_enter_blocking(&USB_MTX);

			uart_write_blocking(UART_ID, USB_BUFFER, USB_POS);
			USB_POS = 0;

			mutex_exit(&USB_MTX);
		}
	}

	return 0;
}
