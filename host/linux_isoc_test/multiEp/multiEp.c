
/*
 * multiEp.c
 * ------------------------
 * File to test mulitple endpoint configurations
 * on and LPC2148 from a linux host. Uses ioctl calls
 * from user-land (not kernel land) execution space.
 *
 */

/* system libs */
#include <usb.h>
#include <linux/capability.h>
#include <linux/usbdevice_fs.h>
#include <asm/byteorder.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

/* utility libs */
#include "lpc_util.h"


int8_t infobuff[1024];

int main(void) {


    int interfaceToClaim = 0;
    int deviceNumber = 0;
    char deviceToOpen[4096];
    int r = findDevice(deviceToOpen, &deviceNumber);
    if( r != 0 ) {
	printf("ERROR: Unable to locate LPCUSB device attached \
                to system (or permissions to /dev/bus/usb are wrong)\n");
	exit(-1);
    } else {
	printf("Found LPCUSB device at %s with device number %d\n", 
               deviceToOpen, deviceNumber);
    }




    return(0);
}


