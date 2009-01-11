#include <usb.h>
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

//Variables for ISOC output
#define ISOC_OUTPUT_PACKET_COUNT 1
struct usbdevfs_urb *myOutURB = NULL;
char isocOutputBuffer[1023];
int isocOutputUniqueID = 1;
uint32_t totalIsocOutBytesSent = 0;
#define ISOC_OUT_ENDPOINT 0x06


//Misc variables
struct usbdevfs_urb *mostRecentReapedURBPtr = NULL;


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


int reapISOC_URB(int fd) {
	int ret, i;
	//printf("-----------------------------------\n");
	//printf("Reaping URB...\n");
	mostRecentReapedURBPtr = NULL;
	
	ret = ioctl(fd, USBDEVFS_REAPURBNDELAY, &mostRecentReapedURBPtr);
	if (ret == -1) {
		//printf("Error %d while reaping URB: %s\n", errno, strerror(errno));
	} else {
		if (mostRecentReapedURBPtr == NULL) {
			//printf("myURB2 is null...\n");
		} else {
			/*
			 Note: DONT REAP URBS FOR ISOC OUT ENDPOINTS 
			 
			ret = ioctl(fd, USBDEVFS_DISCARDURB, mostRecentReapedURBPtr);
			printf("Ret after discarding URB is %d", ret);
			ret = 0;
			printf("\n");
			*/
		}
		
	}
	
	return(ret);
}


int frameNumber = 1;
void writeURBOutISOC(int fd, struct usbdevfs_urb *myOutURB, int urbStructSize, char *stringBuff, void *requestUniqueIDOut, int bufferLength) {
	int ret,i;
	
	memset(myOutURB, 0, urbStructSize);
    myOutURB->type = USBDEVFS_URB_TYPE_ISO;
    myOutURB->flags |= USBDEVFS_URB_ISO_ASAP;
    myOutURB->endpoint = ISOC_OUT_ENDPOINT;
    myOutURB->buffer = stringBuff;
    myOutURB->buffer_length = bufferLength;
    myOutURB->actual_length = 0;
    myOutURB->usercontext = requestUniqueIDOut;
    myOutURB->start_frame = frameNumber;
    frameNumber++;
    myOutURB->number_of_packets = ISOC_OUTPUT_PACKET_COUNT;
    myOutURB->iso_frame_desc[0].length = myOutURB->buffer_length;
    
    
    
	printf("-----------------------------------\n");
	printf("Writing URB to isoc out...\n");
	
    ret = ioctl(fd, USBDEVFS_SUBMITURB, myOutURB);
    if (ret == -1) {
		printf("Error %d while trying to write string to usb device, string is: %s\n", errno, strerror(errno));
	}  else {
		printf("Wrote string to device, ret = %d\n", ret);
		totalIsocOutBytesSent += bufferLength;
	}
    printf("\n");
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
    
 
    int isocOutputURBStructSize = sizeof(struct usbdevfs_urb) + (ISOC_OUTPUT_PACKET_COUNT * sizeof(struct usbdevfs_iso_packet_desc));
    myOutURB = (struct usbdevfs_urb*) malloc(isocOutputURBStructSize);	
    	
	time_t startTime = time(NULL);
	time_t timeDelta;
	

	char ch = 'a';
	for(i = 0; i < sizeof(isocOutputBuffer); i++ ) {
		isocOutputBuffer[i] = ch;
		ch = (ch == 'z' ? 'a' : (ch + 1));
	}
	
	writeURBOutISOC(fd, myOutURB, isocOutputURBStructSize, isocOutputBuffer, &isocOutputUniqueID, sizeof(isocOutputBuffer));
	int q;
	
	for (q = 0; q < 100000; q++) {
		
		//rotatie the buffer so it looks different
		ch = isocOutputBuffer[0];
		for(i = 0; i < sizeof(isocOutputBuffer) - 1; i++ ) {
			isocOutputBuffer[i] = isocOutputBuffer[i+1];
		}
		isocOutputBuffer[sizeof(isocOutputBuffer) - 1] = ch;

		
		 //printf("Sending: '%s'\n", isocOutputBuffer);
		 writeURBOutISOC(fd, myOutURB, isocOutputURBStructSize, isocOutputBuffer, &isocOutputUniqueID, sizeof(isocOutputBuffer));
		 while (reapISOC_URB(fd) < 0) {
			 
		 }
		 
		 printf("Reaped an isoc output URB, EP=0x%X\n", mostRecentReapedURBPtr->endpoint);
		 printf("frame number: %d\n", mostRecentReapedURBPtr->start_frame);
		 
		 
		timeDelta = time(NULL) - startTime;
		if (timeDelta > 0) {
			printf("totalIsocOutBytesSent = %d\n", totalIsocOutBytesSent);
			long isocOutBytesPerSecond = totalIsocOutBytesSent / timeDelta;
			printf("Isoc Out Bytes per second is %d\n", isocOutBytesPerSecond);
		}
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


