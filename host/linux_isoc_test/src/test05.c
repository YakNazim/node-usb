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
#include <time.h>



int8_t infobuff[4096];
int8_t infobuff2[4096];
int8_t infobuff3[4096];



int findDevice(char *destinationDevicePathLocation, int *deviceNumber) {
	DIR *pDIR;
	int i;
	struct dirent *pDirEnt, *pDirEnt2;
	char currentBusDirectory[2048];
	char deviceOfInterest[2048];
	unsigned char deviceConfigData[2048];



	char rootDir[] = "/dev/bus/usb/"; 
	
	/* Open the current directory */

	pDIR = opendir(rootDir);

	if (pDIR == NULL) {
		fprintf( stderr, "%s %d: opendir() failed (%s)\n",
		__FILE__, __LINE__, strerror( errno ));
		exit( -1);
	}

	/* Get each directory entry from pDIR and print its name */

	pDirEnt = readdir(pDIR);
	while (pDirEnt != NULL) {
		//printf("%s\n", pDirEnt->d_name);
		
		if( pDirEnt->d_name[0] == '0' ) {
			strncpy(currentBusDirectory, rootDir, 2048);
			strcat(currentBusDirectory, pDirEnt->d_name);
			DIR *pDIR2 = opendir(currentBusDirectory);
			if (pDIR2 == NULL) {
				fprintf( stderr, "%s %d: opendir() failed (%s)\n",
				__FILE__, __LINE__, strerror( errno ));
				exit( -1);
			}	
			
			pDirEnt2 = readdir(pDIR2);
			while (pDirEnt2 != NULL) {	
				strncpy(deviceOfInterest, currentBusDirectory, 2048);
				strcat(deviceOfInterest, "/");
				strcat(deviceOfInterest, pDirEnt2->d_name);

				if (pDirEnt2->d_name[0] >= '0' && pDirEnt2->d_name[0] <= '9') {
					int fd2 = open(deviceOfInterest, O_ASYNC | O_RDWR);
					int readCount = read(fd2, deviceConfigData, 18);
					if (readCount != 18) {
						printf("ERROR: Unable to read from device %s\n", deviceOfInterest);
						exit(-1);
					} else {
						printf("Opened device %s\n", deviceOfInterest);
					}

					/*
					for(i = 8; i <= 13; i++ ) {
						printf("%X ", deviceConfigData[i]);
					}
					printf("\n");
					*/
					
					if (deviceConfigData[8] == 0xFF && deviceConfigData[9] == 0xFF) {
						if (deviceConfigData[10] == 0x05 && deviceConfigData[11] == 0x00) {
							if (deviceConfigData[12] == 0x00 && deviceConfigData[13] == 0x01) {
								printf("Found LPCUSB device at %s\n", deviceOfInterest);
								
								strcpy(destinationDevicePathLocation, deviceOfInterest);
								*deviceNumber = atoi(pDirEnt2->d_name);
								
								closedir(pDIR2);
								close(fd2);
								closedir(pDIR);
								
								
								return(0);
							}
						}
					}

					close(fd2);
				}
				
				pDirEnt2 = readdir(pDIR2);
			}

			
			closedir(pDIR2);
		}
			
		pDirEnt = readdir(pDIR);
	}

	/* Release the open directory */

	closedir(pDIR);
	
	return(-1);
}

void reapURB(int fd) {
	int ret, i;
	struct usbdevfs_urb *myURBPtr = NULL;
	printf("-----------------------------------\n");
	printf("Reaping URB...\n");
	
	ret = ioctl(fd, USBDEVFS_REAPURBNDELAY, &myURBPtr);
	if (ret == -1) {
		printf("Error %d while reaping URB: %s\n", errno, strerror(errno));
	} else {
		if (myURBPtr == NULL) {
			printf("myURB2 is null...\n");
		} else {
			printf("myURBPtr.status is %d\n", myURBPtr->status);
			printf("myURBPtr.flag is %d\n", myURBPtr->flags);
			printf("myURBPtr.error_count is %d\n", myURBPtr->error_count);
			printf("myURBPtr.number_of_packets is %d\n", myURBPtr->number_of_packets);
			printf("myURBPtr.actual_length is %d\n", myURBPtr->actual_length);
			printf("myURBPtr.usercontext is %d\n", myURBPtr->usercontext);

			if (myURBPtr->endpoint != 0x05) {
				for (i = 0; i < myURBPtr->actual_length; i++) {
					printf("%c", ((char*)myURBPtr->buffer)[i]);
				}
				printf("\n");
			}
			
			ret = ioctl(fd, USBDEVFS_DISCARDURB, myURBPtr);
			printf("Ret after discarding URB is %d", ret);
		}
	}
	
	printf("\n");
	sleep(1);
}

void writeURB(int fd, struct usbdevfs_urb *myOutURB, char *stringBuff, void *requestUniqueIDOut) {
	int ret;
    memset(myOutURB, 0, sizeof(struct usbdevfs_urb));
    myOutURB->type = USBDEVFS_URB_TYPE_BULK;
    myOutURB->endpoint = 0x05;
    myOutURB->buffer = stringBuff;
    myOutURB->buffer_length = strlen(stringBuff);;
    myOutURB->actual_length = myOutURB->buffer_length;
    myOutURB->number_of_packets = 0;
    myOutURB->usercontext = &requestUniqueIDOut;
    
	printf("-----------------------------------\n");
	printf("Writing URB to bulk out...\n");
	
    ret = ioctl(fd, USBDEVFS_SUBMITURB, myOutURB);
    if (ret == -1) {
		printf("Error %d while trying to write string to usb device, string is: %s\n", errno, strerror(errno));
	}  else {
		printf("Wrote string to device, ret = %d\n", ret);
	}
    printf("\n");
    sleep(1);
}





struct usbdevfs_urb *myOutURB = NULL;
struct usbdevfs_urb *myOutURB2 = NULL;
#define URB_ARR_COUNT 16
struct usbdevfs_urb *myOutURBArray[URB_ARR_COUNT];
int urbIdArray[URB_ARR_COUNT];

int packetCount = 32;
int startFrame = 0;


unsigned char isocInputDestinationBuffer[16024000];

void writeURBInISOC(int fd, struct usbdevfs_urb *myOutURB, void *requestUniqueIDOut, int structSize ) {
	int ret,i;
	
	memset(myOutURB, 0, structSize);
    myOutURB->type = USBDEVFS_URB_TYPE_ISO;
    myOutURB->flags |= USBDEVFS_URB_ISO_ASAP;
    myOutURB->endpoint = 0x83;
    myOutURB->buffer = isocInputDestinationBuffer;
    myOutURB->buffer_length = 1023 * packetCount;
    myOutURB->actual_length = 0;
    myOutURB->usercontext = &requestUniqueIDOut;
    myOutURB->start_frame = startFrame;
    
    myOutURB->number_of_packets = packetCount;
    for(i = 0; i < packetCount; i++ ) {
    	myOutURB->iso_frame_desc[i].length = 1023;
    }
    
    
    
	printf("-----------------------------------\n");
	printf("Writing URB to isoc in...\n");
	
    ret = ioctl(fd, USBDEVFS_SUBMITURB, myOutURB);
    if (ret == -1) {
		printf("Error %d while trying to write string to usb device, string is: %s\n", errno, strerror(errno));
	}  else {
		printf("Wrote string to device, ret = %d\n", ret);
		

		printf("myOutURB.status is %d\n", myOutURB->status);
		printf("myOutURB.flag is %d\n", myOutURB->flags);
		printf("myOutURB.error_count is %d\n", myOutURB->error_count);
		printf("myOutURB.number_of_packets is %d\n", myOutURB->number_of_packets);
		printf("myOutURB.actual_length is %d\n", myOutURB->actual_length);
		printf("myOutURB.start_frame is %d\n", myOutURB->start_frame);
		printf("myOutURB.usercontext is %d\n", myOutURB->usercontext);
		printf("myOutURB->iso_frame_desc[0].actual_length is %d\n", myOutURB->iso_frame_desc[0].actual_length);
		printf("myOutURB->iso_frame_desc[0].length is %d\n", myOutURB->iso_frame_desc[0].length);
		printf("myOutURB->iso_frame_desc[0].status is %d\n", myOutURB->iso_frame_desc[0].status);
		
		//for (i = 0; i < myOutURB->iso_frame_desc[0].actual_length; i++) {
			//printf(" 0x%X", ((unsigned char*)myOutURB->buffer)[i]);
		//}
		//printf("\n");
	}
    printf("\n");
    //sleep(1);
}


int totalIsocInBytesReceived = 0;

int reapISOC_URB(int fd) {
	int ret, i;
	struct usbdevfs_urb *myURBPtr = NULL;
	//printf("-----------------------------------\n");
	//printf("Reaping URB...\n");
	
	ret = ioctl(fd, USBDEVFS_REAPURBNDELAY, &myURBPtr);
	if (ret == -1) {
		//printf("Error %d while reaping URB: %s\n", errno, strerror(errno));
	} else {
		if (myURBPtr == NULL) {
			//printf("myURB2 is null...\n");
		} else {
			/*
			printf("myURBPtr.status is %d\n", myURBPtr->status);
			printf("myURBPtr.flag is %d\n", myURBPtr->flags);
			printf("myURBPtr.error_count is %d\n", myURBPtr->error_count);
			printf("myURBPtr.number_of_packets is %d\n", myURBPtr->number_of_packets);
			printf("myURBPtr.start_frame is %d\n", myURBPtr->start_frame);
			printf("myURBPtr.actual_length is %d\n", myURBPtr->actual_length);
			printf("myURBPtr.usercontext is %d\n", myURBPtr->usercontext);
*/
			startFrame += myURBPtr->number_of_packets;
			
			if (myURBPtr->endpoint != 0x05) {
				totalIsocInBytesReceived += myURBPtr->actual_length;
				/*
				for (i = 0; i < myURBPtr->actual_length; i++) {
					printf("0x%X ", ((char*)myURBPtr->buffer)[i]);
				}
				printf("\n");
				*/
			}
			
			/*
			for (i = 0; i < myURBPtr->number_of_packets; i++) {
				printf("myURBPtr->iso_frame_desc[i].actual_length is %d\n", myURBPtr->iso_frame_desc[i].actual_length);
				printf("myURBPtr->iso_frame_desc[i].length is %d\n", myURBPtr->iso_frame_desc[i].length);
				printf("myURBPtr->iso_frame_desc[i].status is %d\n", myURBPtr->iso_frame_desc[i].status);
			}
			*/
			
			ret = ioctl(fd, USBDEVFS_DISCARDURB, myURBPtr);
			printf("Ret after discarding URB is %d", ret);
			ret = 0;
			printf("\n");
		}
		
	}
	
	return(ret);
}







void writeURBOutISOC(int fd, struct usbdevfs_urb *myOutURB, char *stringBuff, void *requestUniqueIDOut) {
	int ret,i;
	int packetCount = 1;
	int structSize = sizeof(struct usbdevfs_urb) + packetCount* sizeof(struct usbdevfs_iso_packet_desc);
	//FIXME dangling
	myOutURB = (struct usbdevfs_urb*)malloc(structSize);
	
	memset(myOutURB, 0, sizeof(struct usbdevfs_urb));
    myOutURB->type = USBDEVFS_URB_TYPE_ISO;
    myOutURB->flags |= USBDEVFS_URB_ISO_ASAP;
    myOutURB->endpoint = 0x06;
    myOutURB->buffer = stringBuff;
    myOutURB->buffer_length = strlen(stringBuff);;
    myOutURB->actual_length = 0;
    myOutURB->usercontext = &requestUniqueIDOut;
    
    myOutURB->number_of_packets = packetCount;
    myOutURB->iso_frame_desc[0].length = myOutURB->buffer_length;
    
    
    
	printf("-----------------------------------\n");
	printf("Writing URB to isoc out...\n");
	
    ret = ioctl(fd, USBDEVFS_SUBMITURB, myOutURB);
    if (ret == -1) {
		printf("Error %d while trying to write string to usb device, string is: %s\n", errno, strerror(errno));
	}  else {
		printf("Wrote string to device, ret = %d\n", ret);
		
		
	}
    printf("\n");
    sleep(1);
}



void readURB(int fd, struct usbdevfs_urb *myInURB, int *requestUniqueID) {
	int ret;
	memset(myInURB, 0, sizeof(struct usbdevfs_urb));

	myInURB->type = USBDEVFS_URB_TYPE_BULK;
	myInURB->endpoint = 0x82;
	myInURB->buffer = infobuff;
	myInURB->buffer_length = 64;
	myInURB->usercontext = requestUniqueID;

	printf("-----------------------------------\n");
	printf("Submiting URB to read data...\n");
	ret = ioctl(fd, USBDEVFS_SUBMITURB, myInURB);
	printf("ret from ioctl after bulk URB is %d\n", ret);
	if (ret == -1) {
		printf("Error %d string is: %s\n", errno, strerror(errno));
	} else {
		printf("myURB.status is %d\n", myInURB->status);
		printf("myURB.flag is %d\n", myInURB->flags);
		printf("myURB.error_count is %d\n", myInURB->error_count);
		printf("myURB.number_of_packets is %d\n", myInURB->number_of_packets);
		printf("myURB.actual_length is %d\n", myInURB->actual_length);
		printf("myURB.usercontext is %d\n", myInURB->usercontext);

	}

	sleep(1);
}


int main(void) {
	
	int i;
	int interfaceToClaim = 0;
	int deviceNumber = 0;
	char deviceToOpen[4096];
	int r = findDevice(deviceToOpen, &deviceNumber);
	if( r != 0 ) {
		printf("ERROR: Unable to locate LPCUSB device attached to system (or permissions to /dev/bus/usb are wrong)\n");
		exit(-1);
	} else {
		printf("Found LPCUSB device at %s with device number %d\n", deviceToOpen, deviceNumber);
	}
	
	int ret;
	char mybuffer[1024];
	mybuffer[0] = 'A';
	
	printf("Trying to open device %s\n", deviceToOpen);
	
    int fd = open(deviceToOpen, O_ASYNC | O_RDWR);
    if( fd == -1 ) {
    	printf("An error occured dirng open of device, errno=%d\n", errno);
    	printf("%s\n", strerror(errno));
    	exit(1);
    }
    
    //Claim interface 1 of the USB device
    printf("Claiming interface...\n");
    ret = ioctl(fd, USBDEVFS_CLAIMINTERFACE, &interfaceToClaim);
    if( ret != 0 ) {
		printf("Error %d while claiming interface, string is: %s\n",errno, strerror(errno));
    }
    
    /*
    struct usbdevfs_connectinfo connInfo;
    connInfo.devnum = deviceNumber;
    connInfo.slow = 2;
    
    ret = ioctl(fd, USBDEVFS_CONNECTINFO, &connInfo);
	printf("ret from ioctl USBDEVFS_CONNECTINFO is %d\n", ret);
	if (ret == -1) {
		printf("Error %d string is: %s\n",errno, strerror(errno));
	} else {
		printf("Slow flag is %d\n", connInfo.slow);
	}
    */
    int q;
    int requestUniqueID = 77;
    int requestUniqueIDOut = 78;
    int requestUniqueIDOut2 = 79;
    
    memset(infobuff3, 0, 4095);
    strncpy(infobuff3, "ABCD", 4095);
    
    int maxData = 64;
    char let = 'a';
    for(q = 0; q <= maxData; q++) {
    	infobuff[q] = let;
    	let++;
    	if( let > 'z' ) {
    		let = 'a';
    	}
    }
    infobuff[maxData] = '\0';
    infobuff[maxData-1] = 'D';
    //infobuff[63] = '\0';
    //infobuff[62] = 'D';
    
    
    //truct usbdevfs_urb myOutURB, myOutURB2, myInURB;

    
    int structSize = sizeof(struct usbdevfs_urb) + packetCount* sizeof(struct usbdevfs_iso_packet_desc);
    myOutURB = (struct usbdevfs_urb*) malloc(structSize);	
    myOutURB2 = (struct usbdevfs_urb*) malloc(structSize);
    	
    
    for(i = 0; i < URB_ARR_COUNT; i++ ) {
    	myOutURBArray[i] = (struct usbdevfs_urb*) malloc(structSize);
    	urbIdArray[i] = i;
    }
    
    
    
    
	time_t startTime = time(NULL);
	time_t timeDelta;
	/*
	for (i = 0; i < URB_ARR_COUNT; i++) {
		printf("===============================================================================\n");
		writeURBInISOC(fd, myOutURBArray[i], &urbIdArray[i], structSize);
	}
	*/
	
	
	for (q = 0; q < 100000; q++) {
		printf("===============================================================================\n");
		writeURBOutISOC(fd, myOutURB, infobuff, &requestUniqueIDOut);
		sleep(1);
		
		reapISOC_URB(fd);
		sleep(1);
		
		/*
		writeURBInISOC(fd, myOutURB2, &requestUniqueIDOut2, structSize);
		while (reapISOC_URB(fd) < 0) {
		}

		timeDelta = time(NULL) - startTime;
		if (timeDelta > 0) {
			long bytesPerSecond = totalIsocInBytesReceived / timeDelta;
			printf("Bytes per second is %d\n", bytesPerSecond);
		}
		*/
		
		
		/*
		for (i = 0; i < URB_ARR_COUNT; i++) {
			printf("===============================================================================\n");
			while (reapISOC_URB(fd) < 0) {
			}

			writeURBInISOC(fd, myOutURBArray[i], &urbIdArray[i], structSize);

			timeDelta = time(NULL) - startTime;
			if (timeDelta > 0) {
				long bytesPerSecond = totalIsocInBytesReceived / timeDelta;
				printf("Bytes per second is %d\n", bytesPerSecond);
			}
		}
		
		*/
	
	
			
		
		
		//reapURB(fd);

		//readURB(fd, &myInURB, &requestUniqueID);
		//reapURB(fd);
	}
	
	//Release the interface
	printf("Releasing interface...\n");
    ret = ioctl(fd, USBDEVFS_RELEASEINTERFACE, &interfaceToClaim);
    if( ret != 0 ) {
		printf("Error %d string is: %s\n",errno, strerror(errno));
    }

	
	
	close(fd);
	
	return(0);
}


