// SPDX-License-Identifier: MIT
/*
 * Copyright (c) 2021 Álvaro Fernández Rojas <noltari@gmail.com>
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 * Copyright (c) 2020 Damien P. George
 */

#if !defined(_TUSB_CONFIG_H_)
#define _TUSB_CONFIG_H_

#include <tusb_option.h>

#define CFG_TUSB_RHPORT0_MODE OPT_MODE_DEVICE

#define CFG_TUD_CDC 2
#define CFG_TUD_CDC_RX_BUFSIZE 256
#define CFG_TUD_CDC_TX_BUFSIZE 256

void usbd_serial_init(void);

#endif /* _TUSB_CONFIG_H_ */
