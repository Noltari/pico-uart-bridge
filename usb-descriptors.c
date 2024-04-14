// SPDX-License-Identifier: MIT
/*
 * Copyright (c) 2021 Álvaro Fernández Rojas <noltari@gmail.com>
 *
 * This file is based on a file originally part of the
 * MicroPython project, http://micropython.org/
 *
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 * Copyright (c) 2019 Damien P. George
 */

#include <hardware/flash.h>
#include <tusb.h>

#define TUD_CDC_BRK_DESCRIPTOR(_itfnum, _stridx, _ep_notif, _ep_notif_size, _epout, _epin, _epsize) \
  /* Interface Associate */\
  8, TUSB_DESC_INTERFACE_ASSOCIATION, _itfnum, 2, TUSB_CLASS_CDC, CDC_COMM_SUBCLASS_ABSTRACT_CONTROL_MODEL, CDC_COMM_PROTOCOL_NONE, 0,\
  /* CDC Control Interface */\
  9, TUSB_DESC_INTERFACE, _itfnum, 0, 1, TUSB_CLASS_CDC, CDC_COMM_SUBCLASS_ABSTRACT_CONTROL_MODEL, CDC_COMM_PROTOCOL_NONE, _stridx,\
  /* CDC Header */\
  5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_HEADER, U16_TO_U8S_LE(0x0120),\
  /* CDC Call */\
  5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_CALL_MANAGEMENT, 0, (uint8_t)((_itfnum) + 1),\
  /* CDC ACM: support line request */\
  4, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_ABSTRACT_CONTROL_MANAGEMENT, 6,\
  /* CDC Union */\
  5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_UNION, _itfnum, (uint8_t)((_itfnum) + 1),\
  /* Endpoint Notification */\
  7, TUSB_DESC_ENDPOINT, _ep_notif, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(_ep_notif_size), 16,\
  /* CDC Data Interface */\
  9, TUSB_DESC_INTERFACE, (uint8_t)((_itfnum)+1), 0, 2, TUSB_CLASS_CDC_DATA, 0, 0, 0,\
  /* Endpoint Out */\
  7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0,\
  /* Endpoint In */\
  7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0


#define DESC_STR_MAX 20

#define USBD_VID 0x2E8A /* Raspberry Pi */
#define USBD_PID 0x000A /* Raspberry Pi Pico SDK CDC */

#define USBD_DESC_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN * CFG_TUD_CDC)
#define USBD_MAX_POWER_MA 500

#define USBD_ITF_CDC_0 0
#define USBD_ITF_CDC_1 2
#define USBD_ITF_MAX 4

#define USBD_CDC_0_EP_CMD 0x81
#define USBD_CDC_1_EP_CMD 0x83

#define USBD_CDC_0_EP_OUT 0x01
#define USBD_CDC_1_EP_OUT 0x03

#define USBD_CDC_0_EP_IN 0x82
#define USBD_CDC_1_EP_IN 0x84

#define USBD_CDC_CMD_MAX_SIZE 8
#define USBD_CDC_IN_OUT_MAX_SIZE 64

#define USBD_STR_0 0x00
#define USBD_STR_MANUF 0x01
#define USBD_STR_PRODUCT 0x02
#define USBD_STR_SERIAL 0x03
#define USBD_STR_SERIAL_LEN 17
#define USBD_STR_CDC 0x04

static const tusb_desc_device_t usbd_desc_device = {
	.bLength = sizeof(tusb_desc_device_t),
	.bDescriptorType = TUSB_DESC_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = TUSB_CLASS_MISC,
	.bDeviceSubClass = MISC_SUBCLASS_COMMON,
	.bDeviceProtocol = MISC_PROTOCOL_IAD,
	.bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
	.idVendor = USBD_VID,
	.idProduct = USBD_PID,
	.bcdDevice = 0x0100,
	.iManufacturer = USBD_STR_MANUF,
	.iProduct = USBD_STR_PRODUCT,
	.iSerialNumber = USBD_STR_SERIAL,
	.bNumConfigurations = 1,
};

static const uint8_t usbd_desc_cfg[USBD_DESC_LEN] = {
	TUD_CONFIG_DESCRIPTOR(1, USBD_ITF_MAX, USBD_STR_0, USBD_DESC_LEN,
		TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, USBD_MAX_POWER_MA),

	TUD_CDC_BRK_DESCRIPTOR(USBD_ITF_CDC_0, USBD_STR_CDC, USBD_CDC_0_EP_CMD,
		USBD_CDC_CMD_MAX_SIZE, USBD_CDC_0_EP_OUT, USBD_CDC_0_EP_IN,
		USBD_CDC_IN_OUT_MAX_SIZE),

	TUD_CDC_BRK_DESCRIPTOR(USBD_ITF_CDC_1, USBD_STR_CDC, USBD_CDC_1_EP_CMD,
		USBD_CDC_CMD_MAX_SIZE, USBD_CDC_1_EP_OUT, USBD_CDC_1_EP_IN,
		USBD_CDC_IN_OUT_MAX_SIZE),
};

static char usbd_serial[USBD_STR_SERIAL_LEN] = "000000000000";

static const char *const usbd_desc_str[] = {
	[USBD_STR_MANUF] = "Raspberry Pi",
	[USBD_STR_PRODUCT] = "Pico",
	[USBD_STR_SERIAL] = usbd_serial,
	[USBD_STR_CDC] = "Board CDC",
};

const uint8_t *tud_descriptor_device_cb(void)
{
	return (const uint8_t *) &usbd_desc_device;
}

const uint8_t *tud_descriptor_configuration_cb(uint8_t index)
{
	return usbd_desc_cfg;
}

const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
	static uint16_t desc_str[DESC_STR_MAX];
	uint8_t len;

	if (index == 0) {
		desc_str[1] = 0x0409;
		len = 1;
	} else {
		const char *str;
		char serial[USBD_STR_SERIAL_LEN];

		if (index >= sizeof(usbd_desc_str) / sizeof(usbd_desc_str[0]))
			return NULL;

		str = usbd_desc_str[index];
		for (len = 0; len < DESC_STR_MAX - 1 && str[len]; ++len)
			desc_str[1 + len] = str[len];
	}

	desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * len + 2);

	return desc_str;
}

void usbd_serial_init(void)
{
	uint8_t id[8];

	flash_get_unique_id(id);

	snprintf(usbd_serial, USBD_STR_SERIAL_LEN, "%02X%02X%02X%02X%02X%02X%02X%02X",
		 id[0], id[1], id[2], id[3], id[4], id[5], id[6], id[7]);
}
