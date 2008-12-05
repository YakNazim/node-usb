
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


static const int interfaceToClaim = 0;

int8_t infobuff[1024];

int main(void) {

    int deviceNumber         = 0;
    int ret;
    char deviceToOpen[4096];
 
    /* Find device */
    ret = findDevice(deviceToOpen, &deviceNumber);

    if( ret != 0 ) {
	printf("ERROR: Unable to locate LPCUSB device attached \
                to system (or permissions to /dev/bus/usb are wrong)\n");
	exit(-1);
    } else {
	printf("Found LPCUSB device at %s with device number %d\n", 
               deviceToOpen, deviceNumber);
    }

    /* Open device */
    printf("Trying to open device %s\n", deviceToOpen);
    	
    int fd = open(deviceToOpen, O_ASYNC | O_RDWR);
    if( fd == -1 ) {
    	printf("An error occured dirng open of device, errno=%d\n", errno);
    	printf("%s\n", strerror(errno));
    	exit(1);
    }

    /* Claim interface X of the USB device */
    printf("Claiming interface...\n");
    ret = ioctl(fd, USBDEVFS_CLAIMINTERFACE, &interfaceToClaim);
    if( ret != 0 ) {
		printf("Error %d while claiming interface, \
                        string is: %s\n",errno, strerror(errno));
    } else {
	printf("Claimed interface %d\n", interfaceToClaim);
    }


    return(0);
}


