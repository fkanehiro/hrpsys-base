/*
 * Linux userspace driver for USB-IO2.0(AKI)
 * Copyright (c) 2014, TABATA Keiichi. All rights reserved.
 */

#ifndef _USBIO_PRIVATE_H_
#define _USBIO_PRIVATE_H_

/*
 * libusb header
 */
#include <usb.h>

/*
 * Device ID for "USB-IO2.0(AKI)"
 */
#define USB_VENDOR	(0x1352)
#define USB_PID		(0x0121)

/*
 * Command packet size
 */
#define CMD_SIZE	(64)

/*
 * Command packet byte offset
 */
#define CMD_OFS_OPCODE	(0)	/* Opcode (==OPCODE_DIO) */
#define CMD_OFS_PORT	(1)	/* Output port number (==PORT_OUT) */
#define CMD_OFS_DATA	(2)	/* Output data */
#define CMD_OFS_SEQ	(63)	/* Command sequence number */

/*
 * Result packet byte offset
 */
#define ACK_OFS_OPCODE	(0)	/* Opcode (==OPCODE_DIO) */
#define ACK_OFS_PORT1	(1)	/* unused (We assume PORT1 is for output) */
#define ACK_OFS_PORT2	(2)	/* Input data of PORT2 */
#define ACK_OFS_SEQ	(63)	/* Command sequence number */

/*
 * Command opcode
 */
#define OPCODE_DIO	(0x20)	/* Digital input and output  */

/*
 * I/O port assignment which we assume.
 */
#define PORT_OUT	(1)	/* Output port (PIN J1[0:7]) */
#define PORT_IN		(2)	/* Input port (PIN J2[0:3], pulled-up) */

/*
 * Endpoint
 */
#define ENDPOINT	(1)

/*
 * Internal data
 */
struct usbio {
	/* libusb enumerated device */
	struct usb_device *ud;

	/* Device parameters */
	int dev_interface;	/* bInterfaceNumber */
	int dev_configuration;	/* bCOnfigurationValue */

	/* libusb acquired device */
	usb_dev_handle *udh;

	/* Saved status of output port for partial pins modification */
	uint8_t output_status;

	/* Command sequence number */
	uint8_t cur_seq_num;
};

#endif
