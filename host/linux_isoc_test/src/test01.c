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



int8_t infobuff[1024];

int main(void) {
	int i;
	char deviceToOpen[] = "/dev/bus/usb/007/006";
	int ret;
	char mybuffer[1024];
	mybuffer[0] = 'A';
	
	printf("Trying to open device %s\n", deviceToOpen);
	
    int fd = open(deviceToOpen, O_SYNC | O_RDWR);
    if( fd == -1 ) {
    	printf("An error occured dirng open of device, errno=%d\n", errno);
    	printf("%s\n", strerror(errno));
    	exit(1);
    }

    ret = read(fd, infobuff, 18);
    printf("Read %d bytes\n", ret);
    
    
    struct usbdevfs_connectinfo connInfo;
    connInfo.devnum = 7;
    connInfo.slow = 2;
    
    ret = ioctl(fd, USBDEVFS_CONNECTINFO, &connInfo);
	printf("ret from ioctl USBDEVFS_CONNECTINFO is %d\n", ret);
	if (ret == -1) {
		printf("Error %d string is: %s\n",errno, strerror(errno));
	} else {
		printf("Slow flag is %d\n", connInfo.slow);
	}
    
    
	struct usbdevfs_bulktransfer testBulk1;
	testBulk1.ep = 0x05;  //bulk output
	testBulk1.len = 1;
	testBulk1.timeout = 1000;//1 second
	testBulk1.data = &mybuffer[0]; 
	
	
	ret = ioctl(fd, USBDEVFS_BULK, &testBulk1);
	
	
	printf("ret from ioctl after bulk output transfer is %d\n", ret);
	if( ret == -1 ) {
		printf("Error %d string is: %s\n", errno, strerror(errno));
	}
	
	
	testBulk1.ep = 0x82; //bulk input
	testBulk1.len = 64;
	testBulk1.timeout = 100;//
	testBulk1.data = &mybuffer[0];

	ret = ioctl(fd, USBDEVFS_BULK, &testBulk1);

	printf("ret from ioctl after bulk input transfer is %d\n", ret);
	if( ret > 0 ) {
		printf("Data read was %d\n", mybuffer[0]);
		for(i = 0; i < ret; i++ ) {
			printf("%c", mybuffer[i]);
		}
		printf("\n");
	}
	if (ret == -1) {
		printf("Error %d string is: %s\n", errno, strerror(errno));
	}
	
	close(fd);
	
	return(0);
}


