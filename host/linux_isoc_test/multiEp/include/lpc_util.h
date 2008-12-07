/*
 * lpc_util.h
 * ------------------------
 * Useful routines to work with linux host side usb
 * and lpc2148.
 */


#ifndef _LPC_UTIL_H
#define _LPC_UTIL_H



/*
 * findDevice
 * --------------
 * find the device entry in /dev/bus/usb
 */
int findDevice(char *destinationDevicePathLocation, int *deviceNumber);
















#endif
