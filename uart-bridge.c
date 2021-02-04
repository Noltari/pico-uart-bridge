// SPDX-License-Identifier: MIT
/*
 * Copyright 2021 Álvaro Fernández Rojas <noltari@gmail.com>
 */

#include <hardware/irq.h>
#include <hardware/structs/sio.h>
#include <hardware/uart.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <tusb.h>

#if !defined(MIN)
#define MIN(a, b) ((a > b) ? b : a)
#endif /* MIN */

#define LED_PIN 25

#define BUFFER_SIZE 64

#define DEF_BIT_RATE 115200
#define DEF_STOP_BITS 1
#define DEF_PARITY 0
#define DEF_DATA_BITS 8

typedef struct {
	uart_inst_t *const inst;
	uint8_t tx_pin;
	uint8_t rx_pin;
} uart_id_t;

typedef struct {
	cdc_line_coding_t usb_lc;
	cdc_line_coding_t uart_lc;
	uint8_t uart_buffer[BUFFER_SIZE];
	uint32_t uart_pos;
	mutex_t uart_mtx;
	uint8_t usb_buffer[BUFFER_SIZE];
	uint32_t usb_pos;
	mutex_t usb_mtx;
} uart_data_t;

const uart_id_t UART_ID[CFG_TUD_CDC] = {
	{
		.inst = uart0,
		.tx_pin = 0,
		.rx_pin = 1,
	}, {
		.inst = uart1,
		.tx_pin = 4,
		.rx_pin = 5,
	}
};

uart_data_t UART_DATA[CFG_TUD_CDC];

static inline uint databits_usb2uart(uint8_t data_bits)
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

static inline uint stopbits_usb2uart(uint8_t stop_bits)
{
	switch (stop_bits) {
		case 2:
			return 2;
		default:
			return 1;
	}
}

void update_uart_cfg(uint8_t itf)
{
	const uart_id_t *ui = &UART_ID[itf];
	uart_data_t *ud = &UART_DATA[itf];

	if (ud->usb_lc.bit_rate != ud->uart_lc.bit_rate) {
		uart_set_baudrate(ui->inst, ud->usb_lc.bit_rate);
		ud->uart_lc.bit_rate = ud->usb_lc.bit_rate;
	}

	if ((ud->usb_lc.stop_bits != ud->uart_lc.stop_bits) ||
	    (ud->usb_lc.parity != ud->uart_lc.parity) ||
	    (ud->usb_lc.data_bits != ud->uart_lc.data_bits)) {
		uart_set_format(ui->inst,
				databits_usb2uart(ud->usb_lc.data_bits),
				stopbits_usb2uart(ud->usb_lc.stop_bits),
				parity_usb2uart(ud->usb_lc.parity));
		ud->uart_lc.data_bits = ud->usb_lc.data_bits;
		ud->uart_lc.parity = ud->usb_lc.parity;
		ud->uart_lc.stop_bits = ud->usb_lc.stop_bits;
	}
}

void usb_read_bytes(uint8_t itf) {
	uint32_t len = tud_cdc_n_available(itf);

	if (len) {
		uart_data_t *ud = &UART_DATA[itf];

		mutex_enter_blocking(&ud->usb_mtx);

		len = MIN(len, BUFFER_SIZE - ud->usb_pos);
		if (len) {
			uint32_t count;

			count = tud_cdc_n_read(itf, ud->usb_buffer, len);
			ud->usb_pos += count;
		}

		mutex_exit(&ud->usb_mtx);
	}
}

void usb_write_bytes(uint8_t itf) {
	uart_data_t *ud = &UART_DATA[itf];

	if (ud->uart_pos) {
		uint32_t count;

		mutex_enter_blocking(&ud->uart_mtx);

		count = tud_cdc_n_write(itf, ud->uart_buffer, ud->uart_pos);
		if (count) {
			ud->uart_pos -= count;
			tud_cdc_n_write_flush(itf);
		}

		mutex_exit(&ud->uart_mtx);
	}
}

void usb_cdc_process(uint8_t itf)
{
	uart_data_t *ud = &UART_DATA[itf];
	int con = tud_cdc_n_connected(itf);

	tud_cdc_n_get_line_coding(itf, &ud->usb_lc);
	usb_read_bytes(itf);
	usb_write_bytes(itf);
}

void core1_entry(void)
{
	tusb_init();

	while (1) {
		int itf;
		int con = 0;

		tud_task();
	
		for (itf = 0; itf < CFG_TUD_CDC; itf++) {
			if (tud_cdc_n_connected(itf)) {
				con = 1;
				usb_cdc_process(itf);
			}
		}

		gpio_put(LED_PIN, con);
	}
}

void uart_read_bytes(uint8_t itf) {
	const uart_id_t *ui = &UART_ID[itf];

	if (uart_is_readable(ui->inst)) {
		uart_data_t *ud = &UART_DATA[itf];

		mutex_enter_blocking(&ud->uart_mtx);

		while (uart_is_readable(ui->inst) &&
			ud->uart_pos < BUFFER_SIZE) {
			ud->uart_buffer[ud->uart_pos] = uart_getc(ui->inst);
			ud->uart_pos++;
		}

		mutex_exit(&ud->uart_mtx);
	}
}

void uart_write_bytes(uint8_t itf) {
	uart_data_t *ud = &UART_DATA[itf];

	if (ud->usb_pos) {
		const uart_id_t *ui = &UART_ID[itf];

		mutex_enter_blocking(&ud->usb_mtx);

		uart_write_blocking(ui->inst, ud->usb_buffer, ud->usb_pos);
		ud->usb_pos = 0;

		mutex_exit(&ud->usb_mtx);
	}
}

void init_uart_data(uint8_t itf) {
	const uart_id_t *ui = &UART_ID[itf];
	uart_data_t *ud = &UART_DATA[itf];

	/* Pinmux */
	gpio_set_function(ui->tx_pin, GPIO_FUNC_UART);
	gpio_set_function(ui->rx_pin, GPIO_FUNC_UART);

	/* USB CDC LC */
	ud->usb_lc.bit_rate = DEF_BIT_RATE;
	ud->usb_lc.data_bits = DEF_DATA_BITS;
	ud->usb_lc.parity = DEF_PARITY;
	ud->usb_lc.stop_bits = DEF_STOP_BITS;

	/* UART LC */
	ud->uart_lc.bit_rate = DEF_BIT_RATE;
	ud->uart_lc.data_bits = DEF_DATA_BITS;
	ud->uart_lc.parity = DEF_PARITY;
	ud->uart_lc.stop_bits = DEF_STOP_BITS;

	/* Buffer */
	ud->uart_pos = 0;
	ud->usb_pos = 0;

	/* Mutex */
	mutex_init(&ud->uart_mtx);
	mutex_init(&ud->usb_mtx);

	/* UART start */
	uart_init(ui->inst, ud->usb_lc.bit_rate);
	uart_set_hw_flow(ui->inst, false, false);
	uart_set_format(ui->inst, databits_usb2uart(ud->usb_lc.data_bits),
			stopbits_usb2uart(ud->usb_lc.stop_bits),
			parity_usb2uart(ud->usb_lc.parity));
}

int main(void)
{
	uint8_t ch;
	int rc;
	int itf;

	for (itf = 0; itf < CFG_TUD_CDC; itf++)
		init_uart_data(itf);

	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);

	multicore_launch_core1(core1_entry);

	while (1) {
		for (itf = 0; itf < CFG_TUD_CDC; itf++) {
			update_uart_cfg(itf);
			uart_read_bytes(itf);
			uart_write_bytes(itf);
		}
	}

	return 0;
}
