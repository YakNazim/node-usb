
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


#define BULK_DATA_MAX             64
#define BULK_TIMEOUT              1000      // 1 second

#define BUFFERSIZE_BULK           256
#define BUFFERSIZE_ISOC           4096


#define BULK_OUT_EP               0x02
#define BULK_IN_EP                0x85



static const int interfaceToClaim = 0;


static char infobuff_bulk_in[BUFFERSIZE_BULK];
static char infobuff_bulk_out[BUFFERSIZE_BULK];
/* static int8_t infobuff_isoc_a[BUFFERSIZE_ISOC]; */
/* static int8_t infobuff_isoc_b[BUFFERSIZE_ISOC]; */


int main(void) {
    struct usbdevfs_bulktransfer bulk_s;

    int deviceNumber         = 0;
    int ret;
    char deviceToOpen[4096];

    int q                    =  0;
    int i                    =  0;
/*     int requestUniqueID      = 77; */
/*     int requestUniqueIDOut   = 78; */
/*     int requestUniqueIDOut2  = 79; */

    int maxData              = BULK_DATA_MAX;

    /*
     * prelude
     */

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
    printf("Claiming interface...%d\n", interfaceToClaim);
    ret = ioctl(fd, USBDEVFS_CLAIMINTERFACE, &interfaceToClaim);
    if( ret != 0 ) {
        printf("Error %d while claiming interface, \
                        string is: %s\n",errno, strerror(errno));
    } else {
        printf("Claimed interface %d\n", interfaceToClaim);
    }



    /*
     * functions
     */
    int j;
    for(j = 0; j < 5; j++ ) {
    
    	memset(infobuff_bulk_in, 0, BUFFERSIZE_BULK);
		strncpy(infobuff_bulk_in, "BADCAB", BUFFERSIZE_BULK);

		memset(infobuff_bulk_out, 0, BUFFERSIZE_BULK);
		strncpy(infobuff_bulk_out, "ABCD", BUFFERSIZE_BULK);

		/* set up data buffer(s) */
		char let = 'a';
		for (q = 0; q <= maxData; q++) {
			infobuff_bulk_out[q] = let;
			let++;
			if (let > 'z') {
				let = 'a';
			}
		}
		infobuff_bulk_out[maxData] = '\0';
		infobuff_bulk_out[maxData-1] = 'D';

		/* write data to device */
		bulk_s.ep = BULK_OUT_EP;
		bulk_s.len = BULK_DATA_MAX;
		bulk_s.timeout = BULK_TIMEOUT;
		bulk_s.data = &infobuff_bulk_out[0];

		ret = ioctl(fd, USBDEVFS_BULK, &bulk_s);

		printf("ret from ioctl after bulk output transfer is %d\n", ret);
		if (ret == -1) {
			printf("Error %d string is: %s\n", errno, strerror(errno));
		}
		printf("Total bulk out transfers is %d\n", j+1);
		sleep(1);

		int k;
		for (k = 0;k < 6;k++) {
			/* read data from device */
			bulk_s.ep = BULK_IN_EP;
			bulk_s.len = BULK_DATA_MAX;
			bulk_s.timeout = BULK_TIMEOUT;
			bulk_s.data = &infobuff_bulk_in[0];

			ret = ioctl(fd, USBDEVFS_BULK, &bulk_s);
			printf("ret from ioctl after bulk input transfer is %d\n", ret);
			if (ret > 0) {
				printf("Data read was %d\n", infobuff_bulk_in[0]);
				for (i = 0; i < ret; i++) {
					printf("%c", infobuff_bulk_in[i]);
				}
				printf("\n");
				break;
			}
			if (ret == -1) {
				printf("Error %d string is: %s\n", errno, strerror(errno));
				break;
			}
			usleep(100000);
		}

		sleep(1);
	}
    
    
    
    
    /*
     * cleanup 
     */

    //Release the interface
    printf("Releasing interface...%d\n", interfaceToClaim);
    ret = ioctl(fd, USBDEVFS_RELEASEINTERFACE, &interfaceToClaim);
    if( ret != 0 ) {
        printf("Error %d string is: %s\n",errno, strerror(errno));
    } else {
        printf("Released interface %d\n", interfaceToClaim);
    }
    

    close(fd);

    return(0);
}


