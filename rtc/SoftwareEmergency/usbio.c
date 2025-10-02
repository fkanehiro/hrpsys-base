/*
 * Linux userspace driver for USB-IO2.0(AKI)
 * Copyright (c) 2014, TABATA Keiichi. All rights reserved.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "usbio.h"
#include "usbio_private.h"

/*
 * Forward declaration
 */
static bool detect_device(usbio_t* hd, int* dev_num);
static void reset_device(usbio_t hd);
static bool open_device(usbio_t hd);
static bool set_configuration(usbio_t hd);
static bool claim_interface(usbio_t hd);
static bool send_command(usbio_t hd, uint8_t send_data, uint8_t send_mask);
static bool recv_response(usbio_t hd, uint8_t *recv_data);
static void dump_usb_error(const char *func_name);


/*
 * Initialization
 */

/*
 * Initialize target device.
 */
bool usbio_init(usbio_t* hd, int* dev_num, bool reset)
{
  int i;
  
  /* Allocate internal data instance. */
  for(i = 0 ; i < MAX_USBIO2_NUM ; i++){
    hd[i] = malloc(sizeof(struct usbio));
    if (hd[i] == NULL)
      return false;
  }
  
  /* Fundamental initialization for libusb. */
  usb_init();
  usb_find_busses();
  usb_find_devices();
  
  *dev_num = 0;
  do {
    /* Detect device. */
    if(!detect_device(hd, dev_num))
      break;
    
    /* Reset device if specified. */
    if (reset) {
      *dev_num = 0;
      for(i = 0 ; i < *dev_num ; i++ )
        reset_device(hd[i]);
      if (!detect_device(hd, dev_num))
	break;
    }
    
    /* Open device. */
    for(i = 0 ; i < *dev_num ; i++){
      if (!open_device(hd[i])){
        fprintf(stderr, "Can't open device #%d\n", i);
        usbio_cleanup(hd[i], true);
      }
    }
    
    /* Success. */
    return true;
  } while(0);
  
  /* Failed. */
  for(i = 0 ; i < *dev_num ; i++)
    usbio_cleanup(hd[i], true);
  return false;
}

/*
 * Detect target device.
 */
bool detect_device(usbio_t* hd, int* dev_num)
{
  struct usb_bus *busses, *bus;
  struct usb_device *dev;
  
  /* Get busses. */
  busses = usb_get_busses();

  /* For each bus. */
  for (bus = busses; bus != NULL; bus = bus->next) {
    /* For each device. */
    for (dev = bus->devices; dev != NULL; dev=dev->next) {
      /* Check device id. */
      if (dev->descriptor.idVendor  == USB_VENDOR &&
	  dev->descriptor.idProduct == USB_PID) {
	/* Found. */
	hd[*dev_num]->ud = dev;
        (*dev_num)++;
      }
    }
  }
  
  if( *dev_num > 0 )
    return true;
  
  /* Not found. */
  fprintf(stderr, "Can't find target device.\n");
  return false;
}

/*
 * Reset target device.
 */
static void reset_device(usbio_t hd)
{
  int res;
  
  /* Open device handle. */
  hd->udh = usb_open(hd->ud);
  if (hd->udh == NULL) {
    dump_usb_error("usb_open");
    return;
  }
  
  /* Reset device. */
  res = usb_reset(hd->udh);
  if (res < 0)
    dump_usb_error("usb_reset");
}

/*
 * Open target device
 */
static bool open_device(usbio_t hd)
{
  hd->dev_configuration = hd->ud->config->bConfigurationValue;
  hd->dev_interface =
    hd->ud->config->interface->altsetting->bInterfaceNumber;
  
  /* Open device handle. */
  hd->udh = usb_open(hd->ud);
  if (hd->udh == NULL) {
    dump_usb_error("usb_open");
    return false;
  }
  
  /* Set configuration. */
  if (!set_configuration(hd))
    return false;
  
  /* Claim interface. */
  if (!claim_interface(hd))
    return false;
  
  /* Succeeded. */
  return true;
}

/*
 * Call usb_set_configuration().
 */
static bool set_configuration(usbio_t hd)
{
  int res;
  
  /* Set configuration. */
  res = usb_set_configuration(hd->udh, hd->dev_configuration);
  if (res == 0)
    return true;	/* Succeeded. */
  dump_usb_error("usb_set_configuration");
  
  /* Recovery. */
  res = usb_detach_kernel_driver_np(hd->udh, hd->dev_interface);
  if (res < 0) {
    dump_usb_error("usb_detach_kernel_driver_np");
    return false;	/* Failed. */
  }
  
  /* Set configuration again. */
  res = usb_set_configuration(hd->udh, hd->dev_configuration);
  if (res == 0)
    return true;	/* Succeeded. */
  dump_usb_error("usb_set_configuration");
  
  /* Failed. */
  return false;
}

/*
 * Call usb_claim_interface().
 */
static bool claim_interface(usbio_t hd)
{
  int res;
  
  /* Claim interface */
  res = usb_claim_interface(hd->udh, hd->dev_interface);
  if (res == 0)
    return true;	/* Succeeded. */
  dump_usb_error("usb_claim_interface");
  
  /* Recovery. */
  res = usb_detach_kernel_driver_np(hd->udh, hd->dev_interface);
  if (res < 0) {
    dump_usb_error("usb_detach_kernel_driver_np");
    return false;	/* Failed. */
  }
  
  /* Claim interface again. */
  res = usb_claim_interface(hd->udh, hd->dev_interface);
  if (res == 0)
    return true;	/* Succeeded. */
  dump_usb_error("usb_claim_interface");
  
  /* Failed. */
  return false;
}

/*
 * Cleanup
 */
void usbio_cleanup(usbio_t hd, bool reset)
{
  int res;
  
  if (hd->udh == NULL)
    return;
  
  /*
   * Detach kernel driver.  This quirk avoids errors when we
   * call usb_claim_interface() again.
   */
  res = usb_detach_kernel_driver_np(hd->udh, hd->dev_interface);
  if (res < 0)
    dump_usb_error("usb_detach_kernel_driver_np");
  
  /*
   * Use usb_reset() instead of usb_close().  If we don't call
   * usb_reset(), the device will not respond until re-plugin.
   */
  if (reset) {
    res = usb_reset(hd->udh);
    if (res < 0)
      dump_usb_error("usb_reset");
  } else {
    res = usb_close(hd->udh);
    if (res < 0)
      dump_usb_error("usb_close");
  }
  
  hd->udh = NULL;
  free(hd);
}

/*
 * I/O operations
 */

/*
 * Read from port2
 */
bool usbio_read(usbio_t hd, uint8_t *recv_data)
{
  return usbio_read_write(hd, recv_data, 0x00, 0x00);
}

/*
 * Write to port1
 */
bool usbio_write(usbio_t hd, uint8_t send_data, uint8_t send_mask)
{
  return usbio_read_write(hd, NULL, send_data, send_mask);
}

/*
 * Read from port2, then write to port1.
 *   recv_data: bit0..3 is set when port2's pin0..3 is open.
 *   send_data: bit0..7 specifies on(1)/off(0) of port1's pin0..7
 */
bool usbio_read_write(usbio_t hd, uint8_t *recv_data, uint8_t send_data,
		      uint8_t send_mask)
{
  /* Send command. */
  if (!send_command(hd, send_data, send_mask))
    return false;	/* Failed. */
  
  /* Receive response. */
  if (!recv_response(hd, recv_data))
    return false;	/* Failed. */
  
  /* Succeeded. */
  return true;
}

/*
 * Send "Digital I/O" command.
 */
static bool send_command(usbio_t hd, uint8_t send_data, uint8_t send_mask)
{
  uint8_t buf[CMD_SIZE];
  int res;
  
  /* Increment command sequence number. */
  hd->cur_seq_num++;
  
  /* Prepare output data. */
  hd->output_status =
    (hd->output_status & (~send_mask)) | /* bits to preserve */
    (send_data & send_mask);		 /* bits to change */
  
  /* Construct "Digital I/O" command packet. */
  memset(buf, 0, CMD_SIZE);
  buf[CMD_OFS_OPCODE] = OPCODE_DIO;
  buf[CMD_OFS_SEQ] = hd->cur_seq_num;
  if (send_mask != 0) {
    buf[CMD_OFS_PORT] = PORT_OUT;
    buf[CMD_OFS_DATA] = hd->output_status;
  }
  
  /* Send the command packet. */
  res = usb_bulk_write(hd->udh, ENDPOINT, buf, CMD_SIZE, 1000);
  if (res < 0 || res != CMD_SIZE) {
    dump_usb_error("usb_bulk_write");
    return false;
  }
  
  /* Succeeded. */
  return true;
}

/*
 * Receive response of "Digital I/O" command.
 */
static bool recv_response(usbio_t hd, uint8_t *recv_data)
{
  uint8_t buf[CMD_SIZE];
  int res, retry = 20, i;
  
  /* Wait for the response. */
  memset(buf, 0, CMD_SIZE);
  while (retry > 0) { 
    res = usb_bulk_read(hd->udh, 1, buf, CMD_SIZE, 100);
    if (res == CMD_SIZE && buf[CMD_OFS_SEQ] == hd->cur_seq_num)
      break;	/* Succeeded. */
    
    printf("res=%d, cur_seq_num=%d, seq=%d\n", res,
	   hd->cur_seq_num, buf[CMD_OFS_SEQ]);
    dump_usb_error("usb_bulk_read");
    usleep(1000);
    retry--;
  }
  if (retry == 0)
    return false;	/* Failed. */
  
  /* Dump response. */
#if 0
  for (i = 0; i < CMD_SIZE; i++)
    printf("%02x ", buf[i]);
  printf("\n");
#endif
  
  /* Set result data if recv_data is non-null. */
  if (recv_data != NULL){
    *recv_data = buf[2];
  }
  
  /* Succeeded. */
  return true;
}

/*
 * Utility
 */

/*
 * Print libusb error message
 */
static void
dump_usb_error(const char *func_name)
{
  char *msg = usb_strerror();
  printf("Error: %s(): %s\n", func_name, msg);
}
