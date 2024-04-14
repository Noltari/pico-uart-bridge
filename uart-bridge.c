// SPDX-License-Identifier: MIT
/*
 * Copyright 2021 Álvaro Fernández Rojas <noltari@gmail.com>
 * Cleanup/modifications Copyright 2023 Andrew J. Kroll <xxxajk at gmail>
 *
 */

#include <hardware/irq.h>
#include <hardware/structs/sio.h>
#include <hardware/uart.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <string.h>
#include <tusb.h>

#if !defined(MIN)
#define MIN(a, b) ((a > b) ? b : a)
#endif /* MIN */

#define SYS_LED_ACTIVE 25

#define BUFFER_SIZE 2560

#define LED_TIMEOUT 10

#define UART0_TX 0
#define UART0_RX 1
#define UART0_LED_TX 19 // not used for UDPI
#define UART0_LED_RX 18

#define UART1_TX 4
#define UART1_RX 5
#define UART1_LED_TX 17
#define UART1_LED_RX 16


#define DEF_BIT_RATE 115200
#define DEF_STOP_BITS 1
#define DEF_PARITY 0
#define DEF_DATA_BITS 8

typedef struct {
        uart_inst_t * const inst;
        uint irq;
        void *irq_fn;
        uint8_t tx_pin;
        uint8_t rx_pin;
        uint8_t tx_led;
        uint8_t rx_led;
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
        volatile uint8_t countdown_LED_TX;
        volatile uint8_t countdown_LED_RX;
        critical_section_t spinlock_LED_TX;
        critical_section_t spinlock_LED_RX;
} uart_data_t;

void uart0_irq_fn(void);
void uart1_irq_fn(void);

const uart_id_t UART_ID[CFG_TUD_CDC] = {
        {
                .inst = uart0,
                .irq = UART0_IRQ,
                .irq_fn = &uart0_irq_fn,
                .tx_pin = UART0_TX,
                .rx_pin = UART0_RX,
                .tx_led = UART0_LED_TX,
                .rx_led = UART0_LED_RX,
        },
        {
                .inst = uart1,
                .irq = UART1_IRQ,
                .irq_fn = &uart1_irq_fn,
                .tx_pin = UART1_TX,
                .rx_pin = UART1_RX,
                .tx_led = UART1_LED_TX,
                .rx_led = UART1_LED_RX,
        }
};

uart_data_t UART_DATA[CFG_TUD_CDC];
struct repeating_timer stimulate;
volatile bool ready = false;

void init_led(uint8_t p) {
        gpio_init(p);
        gpio_set_dir(p, GPIO_OUT);
        gpio_put(p, 0);
}

static inline uint databits_usb2uart(uint8_t data_bits) {
        switch(data_bits) {
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

static inline uart_parity_t parity_usb2uart(uint8_t usb_parity) {
        switch(usb_parity) {
                case 1:
                        return UART_PARITY_ODD;
                case 2:
                        return UART_PARITY_EVEN;
                default:
                        return UART_PARITY_NONE;
        }
}

static inline uint stopbits_usb2uart(uint8_t stop_bits) {
        switch(stop_bits) {
                case 2:
                        return 2;
                default:
                        return 1;
        }
}

void update_uart_cfg(uint8_t itf) {
        const uart_id_t *ui = &UART_ID[itf];
        uart_data_t *ud = &UART_DATA[itf];

        mutex_enter_blocking(&ud->lc_mtx);

        if(ud->usb_lc.bit_rate != ud->uart_lc.bit_rate) {
                uart_set_baudrate(ui->inst, ud->usb_lc.bit_rate);
                ud->uart_lc.bit_rate = ud->usb_lc.bit_rate;
        }

        if((ud->usb_lc.stop_bits != ud->uart_lc.stop_bits) || (ud->usb_lc.parity != ud->uart_lc.parity) || (ud->usb_lc.data_bits != ud->uart_lc.data_bits)) {
                uart_set_format(ui->inst, databits_usb2uart(ud->usb_lc.data_bits), stopbits_usb2uart(ud->usb_lc.stop_bits), parity_usb2uart(ud->usb_lc.parity));
                ud->uart_lc.data_bits = ud->usb_lc.data_bits;
                ud->uart_lc.parity = ud->usb_lc.parity;
                ud->uart_lc.stop_bits = ud->usb_lc.stop_bits;
        }

        mutex_exit(&ud->lc_mtx);
}

void usb_read_bytes(uint8_t itf) {
        uart_data_t *ud = &UART_DATA[itf];
        uint32_t len = tud_cdc_n_available(itf);

        if(len && mutex_try_enter(&ud->usb_mtx, NULL)) {
                len = MIN(len, BUFFER_SIZE - ud->usb_pos);
                if(len) {
                        uint32_t count;
                        count = tud_cdc_n_read(itf, &ud->usb_buffer[ud->usb_pos], len);
                        ud->usb_pos += count;
                }

                mutex_exit(&ud->usb_mtx);
        }
}

void usb_write_bytes(uint8_t itf) {
        uart_data_t *ud = &UART_DATA[itf];

        if(ud->uart_pos) {
                if(mutex_try_enter(&ud->uart_mtx, NULL)) {
                        uint32_t count = tud_cdc_n_write(itf, ud->uart_buffer, ud->uart_pos);
                        // horrible! should use ring buffers!!
                        if(count < ud->uart_pos) {
                                memmove(ud->uart_buffer, &ud->uart_buffer[count], ud->uart_pos - count);
                        }
                        ud->uart_pos -= count;
                        mutex_exit(&ud->uart_mtx);

                        if(count)
                                tud_cdc_n_write_flush(itf);
                }
        }
}

void tud_cdc_send_break_cb(uint8_t itf, uint16_t duration_ms) {
        const uart_id_t *ui = &UART_ID[itf];
        uart_data_t *ud = &UART_DATA[itf];

        // is mutex for tx even needed??
        //mutex_enter_blocking(&ud->lc_mtx);

        if(duration_ms == 0xffff) {
                uart_set_break(ui->inst, true);
        } else if(duration_ms == 0x0000) {
                uart_set_break(ui->inst, false);
        } else {
                // should be correct for non-compliant stacks?
                uart_set_break(ui->inst, true);
                sleep_ms(duration_ms);
                uart_set_break(ui->inst, false);
        }
        //mutex_exit(&ud->lc_mtx);
}

void usb_cdc_process(uint8_t itf) {
        uart_data_t *ud = &UART_DATA[itf];

        mutex_enter_blocking(&ud->lc_mtx);
        tud_cdc_n_get_line_coding(itf, &ud->usb_lc);
        mutex_exit(&ud->lc_mtx);

        usb_read_bytes(itf);
        usb_write_bytes(itf);
}

void core1_entry(void) {
        tusb_init();
        ready = true;

        while(1) {
                int itf;
                tud_task();

                if(tud_ready()) { // we need to ignore DTR on the CDC side
                        gpio_put(SYS_LED_ACTIVE, 1);
                        for(itf = 0; itf < CFG_TUD_CDC; itf++) {
                                usb_cdc_process(itf);
                        }
                } else {
                        gpio_put(SYS_LED_ACTIVE, 0);
                }
        }
}

void stimulate_status(uint8_t itf) {
        const uart_id_t *ui = &UART_ID[itf];
        uart_data_t *ud = &UART_DATA[itf];
        critical_section_enter_blocking(&ud->spinlock_LED_RX);
        if(ud->countdown_LED_RX == LED_TIMEOUT) {
                gpio_put(ui->rx_led, 1);
        }
        if(ud->countdown_LED_RX != 0) {
                ud->countdown_LED_RX--;
                if(ud->countdown_LED_RX == 0) {
                        gpio_put(ui->rx_led, 0);
                }
        }
        critical_section_exit(&ud->spinlock_LED_RX);

        critical_section_enter_blocking(&ud->spinlock_LED_TX);
        if(ud->countdown_LED_TX == LED_TIMEOUT) {
                gpio_put(ui->tx_led, 1);
        }
        if(ud->countdown_LED_TX != 0) {
                ud->countdown_LED_TX--;
                if(ud->countdown_LED_TX == 0) {
                        gpio_put(ui->tx_led, 0);
                }
        }
        critical_section_exit(&ud->spinlock_LED_TX);
}

bool update_status(struct repeating_timer *t) {
        for(uint8_t itf = 0; itf < CFG_TUD_CDC; itf++) {
                stimulate_status(itf);
        }
        return true;
}

static inline void uart_read_bytes(uint8_t itf) {
        uart_data_t *ud = &UART_DATA[itf];
        const uart_id_t *ui = &UART_ID[itf];

        if(uart_is_readable(ui->inst)) {

                critical_section_enter_blocking(&ud->spinlock_LED_RX);
                ud->countdown_LED_RX = LED_TIMEOUT;
                critical_section_exit(&ud->spinlock_LED_RX);

                mutex_enter_blocking(&ud->uart_mtx);

                if(ud->uart_pos < BUFFER_SIZE) {
                        ud->uart_buffer[ud->uart_pos] = uart_getc(ui->inst);
                        ud->uart_pos++;
                } else {
                        uart_getc(ui->inst); // drop it on the floor
                }
                mutex_exit(&ud->uart_mtx);
        }
}

void uart_write_bytes(uint8_t itf) {
        uart_data_t *ud = &UART_DATA[itf];

        if(ud->usb_pos && mutex_try_enter(&ud->usb_mtx, NULL)) {
                const uart_id_t *ui = &UART_ID[itf];

                // horrible! should use ring buffers!!
                if(uart_is_writable(ui->inst)) { // && count < ud->usb_pos) {
                        critical_section_enter_blocking(&ud->spinlock_LED_TX);
                        ud->countdown_LED_TX = LED_TIMEOUT;
                        critical_section_exit(&ud->spinlock_LED_TX);
                        uart_putc_raw(ui->inst, ud->usb_buffer[0]);
                        if(ud->usb_pos > 1) {
                                memmove(ud->usb_buffer, &ud->usb_buffer[1], ud->usb_pos - 1);
                        }
                        ud->usb_pos--;
                }
                mutex_exit(&ud->usb_mtx);
        }
}

void uart0_irq_fn(void) {
        uart_read_bytes(0);
}

void uart1_irq_fn(void) {
        uart_read_bytes(1);
}

void init_uart_data(uint8_t itf) {
        const uart_id_t *ui = &UART_ID[itf];
        uart_data_t *ud = &UART_DATA[itf];

        init_led(ui->tx_led);
        init_led(ui->rx_led);
        ud->countdown_LED_TX = 0;
        ud->countdown_LED_RX = 0;
        critical_section_init(&ud->spinlock_LED_TX);
        critical_section_init(&ud->spinlock_LED_RX);

        /* Pinmux */
        gpio_set_function(ui->tx_pin, GPIO_FUNC_UART);
        gpio_set_function(ui->rx_pin, GPIO_FUNC_UART);
        gpio_pull_up(ui->rx_pin); // important missed detail, prevents connection glitches

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
        uart_init(ui->inst, ud->usb_lc.bit_rate);
        uart_set_hw_flow(ui->inst, false, false);
        uart_set_format(ui->inst, databits_usb2uart(ud->usb_lc.data_bits),
                stopbits_usb2uart(ud->usb_lc.stop_bits),
                parity_usb2uart(ud->usb_lc.parity));
        uart_set_fifo_enabled(ui->inst, false);
        uart_set_translate_crlf(ui->inst, false);
        /* UART RX Interrupt */
        irq_set_exclusive_handler(ui->irq, ui->irq_fn);
}

void start_uarts() {
        uint8_t itf;
        for(itf = 0; itf < CFG_TUD_CDC; itf++) {
                init_uart_data(itf);
        }

        // init led stimulator
        add_repeating_timer_ms(1, update_status, NULL, &stimulate);

        // enable ISRs
        for(itf = 0; itf < CFG_TUD_CDC; itf++) {
                const uart_id_t *ui = &UART_ID[itf];
                irq_set_enabled(ui->irq, true);
                uart_set_irq_enables(ui->inst, true, false);
        }
}

int main(void) {
        int itf;

        set_sys_clock_khz(125000, false);

        multicore_reset_core1();

        init_led(SYS_LED_ACTIVE);
        usbd_serial_init();

        start_uarts();

        multicore_launch_core1(core1_entry);
        do {
                sleep_us(1);
        } while(!ready);

        while(1) {
                if(tud_ready()) {
                        for(itf = 0; itf < CFG_TUD_CDC; itf++) {
                                update_uart_cfg(itf);
                                uart_write_bytes(itf);
                        }
                }
        }

        return 0;
}
