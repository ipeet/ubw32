/******************************************************************************
 * uart_api.c
 * Copyright Complicated: Based on work by Microchip Inc, Brian Schmalz, and
 * Iain Peet
 *
 * Provides functions for reading from / writing to a virtual usb serial 
 * device.  Loosley based on the linux read() and write() system calls. The
 * USB framework is driven by polling; the provided pump_usb() function will
 * need to be called relativley frequently (~1ms) for full data rate.
 ******************************************************************************/
#ifndef usb_api_h_
#define usb_api_h_

#include "file_io.h"

/* Begins the USB configuration process.
 * pump_usb() will need to be called frequently from now on.
 * @param devname  The device name to mount with while USB is configured.  
 * @param devflags Any device-level flags (e.g O_ECHO) to apply. */
void init_usb(const char *devname, int devflags);

#endif

