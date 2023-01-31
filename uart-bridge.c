// SPDX-License-Identifier: MIT
/*
 * Copyright 2021 Álvaro Fernández Rojas <noltari@gmail.com>
 */

#include <hardware/clocks.h>
#include <hardware/irq.h>
#include <hardware/resets.h>
#include <hardware/structs/sio.h>
#include <hardware/uart.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <string.h>
#include <tusb.h>

#if !defined(MIN)
#define MIN(a, b) ((a > b) ? b : a)
#endif /* MIN */

#define LED_PIN 25

#define BUFFER_SIZE 2560

#define DEF_BIT_RATE 115200
#define DEF_STOP_BITS 1
#define DEF_PARITY 0
#define DEF_DATA_BITS 8

typedef struct {
	uart_inst_t *const inst;
	uint irq;
	void *irq_fn;
	uint8_t tx_pin;
	uint8_t rx_pin;
} uart_id_t;

typedef struct {
	cdc_line_coding_t usb_lc;
	cdc_line_coding_t uart_lc;
	mutex_t lc_mtx;
	uint8_t uart_buffer[BUFFER_SIZE];
	uint32_t uart_pos;
	mutex_t uart_mtx;
	uint8_t usb_buffer[BUFFER_SIZE];
	uint32_t usb_pos;
	mutex_t usb_mtx;
} uart_data_t;

void uart0_irq_fn(void);
void uart1_irq_fn(void);

const uart_id_t UART_ID[CFG_TUD_CDC] = {
	{
		.inst = uart0,
		.irq = UART0_IRQ,
		.irq_fn = &uart0_irq_fn,
		.tx_pin = 16,
		.rx_pin = 17,
	}, {
		.inst = uart1,
		.irq = UART1_IRQ,
		.irq_fn = &uart1_irq_fn,
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

uint32_t uart_disable_before_lcr_write(uart_inst_t *uart)
{
    // Notes from PL011 reference manual:
    //
    // - Before writing LCR, must disable UART and wait for current TX + RX
    //   char to finish
    //
    // - There is a BUSY flag which waits for the current TX char, but this is
    //   OR'd with TX FIFO !FULL, so not usable when FIFOs are enabled and
    //   potentially nonempty
    //
    // - FIFOs can't be set to disabled whilst a character is in progress
    //   (else "FIFO integrity is not guaranteed")
    //
    // Combination of these means there is no general way to halt and poll for
    // end of TX character, if FIFOs may be enabled. Either way, there is no
    // way to poll for end of RX character.
    //
    // So... we're going to insert a 15 baud period delay before changing
    // settings (comfortably higher than max data + start + stop + parity).
    // Anything else would require API changes to permit a non-enabled UART
    // state after init() where settings can be changed safely.

    uint32_t cr_save = uart_get_hw(uart)->cr;
    hw_clear_bits(&uart_get_hw(uart)->cr,
        UART_UARTCR_UARTEN_BITS | UART_UARTCR_TXE_BITS | UART_UARTCR_RXE_BITS);

    uint32_t current_ibrd = uart_get_hw(uart)->ibrd;
    uint32_t current_fbrd = uart_get_hw(uart)->fbrd;
    uint64_t baud_period_usec = 1u +
        ((uint64_t)64 * current_ibrd + current_fbrd) /
        (4u * clock_get_hz(clk_peri));

    busy_wait_us(15 * baud_period_usec);

    return cr_save;
}

void _uart_set_fifo_enabled(uart_inst_t *uart, bool enabled) {
    bool was_enabled = uart_is_enabled(uart);
    uint32_t cr_save;
    if (was_enabled)
        cr_save = uart_disable_before_lcr_write(uart);

    hw_write_masked(&uart_get_hw(uart)->lcr_h,
                   (bool_to_bit(enabled) << UART_UARTLCR_H_FEN_LSB),
                   UART_UARTLCR_H_FEN_BITS);

    if (was_enabled)
        uart_get_hw(uart)->cr = cr_save;
}

uint _uart_set_baudrate(uart_inst_t *uart, uint baudrate) {
    invalid_params_if(UART, baudrate == 0);
    uint32_t baud_rate_div = (8 * clock_get_hz(clk_peri) / baudrate);
    uint32_t baud_ibrd = baud_rate_div >> 7;
    uint32_t baud_fbrd;

    if (baud_ibrd == 0) {
        baud_ibrd = 1;
        baud_fbrd = 0;
    } else if (baud_ibrd >= 65535) {
        baud_ibrd = 65535;
        baud_fbrd = 0;
    }  else {
        baud_fbrd = ((baud_rate_div & 0x7f) + 1) / 2;
    }

    // Need to cleanly disable UART before touching LCR
    bool was_enabled = uart_is_enabled(uart);
    uint32_t cr_save;
    if (was_enabled)
        cr_save = uart_disable_before_lcr_write(uart);

    uart_get_hw(uart)->ibrd = baud_ibrd;
    uart_get_hw(uart)->fbrd = baud_fbrd;
    // PL011 needs a (dummy) LCR_H write to latch in the divisors. We don't
    // want to actually change LCR_H contents here.
    hw_set_bits(&uart_get_hw(uart)->lcr_h, 0);

    // Re-enable using saved control register value
    if (was_enabled)
        uart_get_hw(uart)->cr = cr_save;

    // See datasheet
    return (4 * clock_get_hz(clk_peri)) / (64 * baud_ibrd + baud_fbrd);
}

void _uart_set_format(uart_inst_t *uart, uint data_bits, uint stop_bits, uart_parity_t parity) {
    invalid_params_if(UART, data_bits < 5 || data_bits > 8);
    invalid_params_if(UART, stop_bits != 1 && stop_bits != 2);
    invalid_params_if(UART, parity != UART_PARITY_NONE && parity != UART_PARITY_EVEN && parity != UART_PARITY_ODD);

    bool was_enabled = uart_is_enabled(uart);
    uint32_t cr_save;
    if (was_enabled)
        cr_save = uart_disable_before_lcr_write(uart);

    hw_write_masked(&uart_get_hw(uart)->lcr_h,
                   ((data_bits - 5u) << UART_UARTLCR_H_WLEN_LSB) |
                   ((stop_bits - 1u) << UART_UARTLCR_H_STP2_LSB) |
                   (bool_to_bit(parity != UART_PARITY_NONE) << UART_UARTLCR_H_PEN_LSB) |
                   (bool_to_bit(parity == UART_PARITY_EVEN) << UART_UARTLCR_H_EPS_LSB),
                   UART_UARTLCR_H_WLEN_BITS |
                   UART_UARTLCR_H_STP2_BITS |
                   UART_UARTLCR_H_PEN_BITS |
                   UART_UARTLCR_H_EPS_BITS);

    if (was_enabled)
        uart_get_hw(uart)->cr = cr_save;
}

static inline void uart_reset(uart_inst_t *uart) {
    invalid_params_if(UART, uart != uart0 && uart != uart1);
    reset_block(uart_get_index(uart) ? RESETS_RESET_UART1_BITS : RESETS_RESET_UART0_BITS);
}

static inline void uart_unreset(uart_inst_t *uart) {
    invalid_params_if(UART, uart != uart0 && uart != uart1);
    unreset_block_wait(uart_get_index(uart) ? RESETS_RESET_UART1_BITS : RESETS_RESET_UART0_BITS);
}

uint _uart_init(uart_inst_t *uart, uint baudrate) {
    invalid_params_if(UART, uart != uart0 && uart != uart1);

    if (clock_get_hz(clk_peri) == 0)
        return 0;

    uart_reset(uart);
    uart_unreset(uart);

#if PICO_UART_ENABLE_CRLF_SUPPORT
    uart_set_translate_crlf(uart, PICO_UART_DEFAULT_CRLF);
#endif

    // Any LCR writes need to take place before enabling the UART
    uint baud = _uart_set_baudrate(uart, baudrate);
    _uart_set_format(uart, 8, 1, UART_PARITY_NONE);

    // Enable FIFOs (must be before setting UARTEN, as this is an LCR access)
    hw_set_bits(&uart_get_hw(uart)->lcr_h, UART_UARTLCR_H_FEN_BITS);
    // Enable the UART, both TX and RX
    uart_get_hw(uart)->cr = UART_UARTCR_UARTEN_BITS | UART_UARTCR_TXE_BITS | UART_UARTCR_RXE_BITS;
    // Always enable DREQ signals -- no harm in this if DMA is not listening
    uart_get_hw(uart)->dmacr = UART_UARTDMACR_TXDMAE_BITS | UART_UARTDMACR_RXDMAE_BITS;

    return baud;
}

void update_uart_cfg(uint8_t itf)
{
	const uart_id_t *ui = &UART_ID[itf];
	uart_data_t *ud = &UART_DATA[itf];

	mutex_enter_blocking(&ud->lc_mtx);

	if (ud->usb_lc.bit_rate != ud->uart_lc.bit_rate) {
		_uart_set_baudrate(ui->inst, ud->usb_lc.bit_rate);
		ud->uart_lc.bit_rate = ud->usb_lc.bit_rate;
	}

	if ((ud->usb_lc.stop_bits != ud->uart_lc.stop_bits) ||
	    (ud->usb_lc.parity != ud->uart_lc.parity) ||
	    (ud->usb_lc.data_bits != ud->uart_lc.data_bits)) {
		_uart_set_format(ui->inst,
				 databits_usb2uart(ud->usb_lc.data_bits),
				 stopbits_usb2uart(ud->usb_lc.stop_bits),
				 parity_usb2uart(ud->usb_lc.parity));
		ud->uart_lc.data_bits = ud->usb_lc.data_bits;
		ud->uart_lc.parity = ud->usb_lc.parity;
		ud->uart_lc.stop_bits = ud->usb_lc.stop_bits;
	}

	mutex_exit(&ud->lc_mtx);
}

void usb_read_bytes(uint8_t itf)
{
	uart_data_t *ud = &UART_DATA[itf];
	uint32_t len = tud_cdc_n_available(itf);

	if (len &&
	    mutex_try_enter(&ud->usb_mtx, NULL)) {
		len = MIN(len, BUFFER_SIZE - ud->usb_pos);
		if (len) {
			uint32_t count;

			count = tud_cdc_n_read(itf, &ud->usb_buffer[ud->usb_pos], len);
			ud->usb_pos += count;
		}

		mutex_exit(&ud->usb_mtx);
	}
}

void usb_write_bytes(uint8_t itf)
{
	uart_data_t *ud = &UART_DATA[itf];

	if (ud->uart_pos &&
	    mutex_try_enter(&ud->uart_mtx, NULL)) {
		uint32_t count;

		count = tud_cdc_n_write(itf, ud->uart_buffer, ud->uart_pos);
		if (count < ud->uart_pos)
			memmove(ud->uart_buffer, &ud->uart_buffer[count],
			       ud->uart_pos - count);
		ud->uart_pos -= count;

		mutex_exit(&ud->uart_mtx);

		if (count)
			tud_cdc_n_write_flush(itf);
	}
}

void usb_cdc_process(uint8_t itf)
{
	uart_data_t *ud = &UART_DATA[itf];

	mutex_enter_blocking(&ud->lc_mtx);
	tud_cdc_n_get_line_coding(itf, &ud->usb_lc);
	mutex_exit(&ud->lc_mtx);

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

static inline void uart_read_bytes(uint8_t itf)
{
	uart_data_t *ud = &UART_DATA[itf];
	const uart_id_t *ui = &UART_ID[itf];

	if (uart_is_readable(ui->inst)) {
		mutex_enter_blocking(&ud->uart_mtx);

		while (uart_is_readable(ui->inst) &&
		       (ud->uart_pos < BUFFER_SIZE)) {
			ud->uart_buffer[ud->uart_pos] = uart_getc(ui->inst);
			ud->uart_pos++;
		}

		mutex_exit(&ud->uart_mtx);
	}
}

void uart0_irq_fn(void)
{
	uart_read_bytes(0);
}

void uart1_irq_fn(void)
{
	uart_read_bytes(1);
}

void uart_write_bytes(uint8_t itf)
{
	uart_data_t *ud = &UART_DATA[itf];

	if (ud->usb_pos &&
	    mutex_try_enter(&ud->usb_mtx, NULL)) {
		const uart_id_t *ui = &UART_ID[itf];
		uint32_t count = 0;

		while (uart_is_writable(ui->inst) &&
		       count < ud->usb_pos) {
			uart_putc_raw(ui->inst, ud->usb_buffer[count]);
			count++;
		}

		if (count < ud->usb_pos)
			memmove(ud->usb_buffer, &ud->usb_buffer[count],
			       ud->usb_pos - count);
		ud->usb_pos -= count;

		mutex_exit(&ud->usb_mtx);
	}
}

void init_uart_data(uint8_t itf)
{
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
	mutex_init(&ud->lc_mtx);
	mutex_init(&ud->uart_mtx);
	mutex_init(&ud->usb_mtx);

	/* UART start */
	_uart_init(ui->inst, ud->usb_lc.bit_rate);
	uart_set_hw_flow(ui->inst, false, false);
	_uart_set_format(ui->inst, databits_usb2uart(ud->usb_lc.data_bits),
			 stopbits_usb2uart(ud->usb_lc.stop_bits),
			 parity_usb2uart(ud->usb_lc.parity));
	_uart_set_fifo_enabled(ui->inst, false);

	/* UART RX Interrupt */
	irq_set_exclusive_handler(ui->irq, ui->irq_fn);
	irq_set_enabled(ui->irq, true);
	uart_set_irq_enables(ui->inst, true, false);
}

int main(void)
{
	int itf;

	set_sys_clock_khz(250000, false);

	usbd_serial_init();

	for (itf = 0; itf < CFG_TUD_CDC; itf++)
		init_uart_data(itf);

	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);

	multicore_launch_core1(core1_entry);

	while (1) {
		for (itf = 0; itf < CFG_TUD_CDC; itf++) {
			update_uart_cfg(itf);
			uart_write_bytes(itf);
		}
	}

	return 0;
}
