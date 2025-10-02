//gcc -o usbio main.c usbio.c -lusb

/*
 * Linux userspace driver for USB-IO2.0(AKI)
 * Copyright (c) 2014, TABATA Keiichi. All rights reserved.
 */

#include <stdio.h>
#include "usbio.h"

/*
 * Example usage
 */

void setLEDStatus(usbio_t hd, int on)
{
  usbio_write(hd, on, 0x01);
}

int
main(int argc, char **argv)
{
  int n_dev = 0; 
  uint8_t recv;
  usbio_t hd[MAX_USBIO2_NUM];
  
  printf("initialize\n");
  
  /* Open target device. */
  bool ret = usbio_init(hd, &n_dev, false);
  if (!ret || (n_dev == 0) )
    return 1;
  
  printf("number of device = %d\n", n_dev);
  
  bool isFoundDevice = false;
  int usb_id = 0;
  for(int i = 0 ; i < n_dev ; i++ ){
    printf("usb_read #%d\n", i);
    usbio_read(hd[i], &recv);
    int id = (recv & 0x0e) >> 1;
    
    printf("id = %d\n", id);
    if( id != 7 )
      usbio_cleanup(hd[i], true);
    else{
      isFoundDevice = true;
      usb_id = i;
    }
  }
  
  if( isFoundDevice ){
    /* Digital I/O. */
    usbio_read(hd[usb_id], &recv);
    
    setLEDStatus(hd[usb_id], recv);
    
    printf("Emergency button = %d\n", recv & 0x01); 
    
    /* Close target device. */
    usbio_cleanup(hd[usb_id], true);
  }
  else
    printf("Can't find Emergency button\n"); 
  
  return 0;
}
