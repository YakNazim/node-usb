/*
 * lpc_util.c
 * ------------------------
 * Useful routines to work with linux host side usb
 * and lpc2148.
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <usb.h>
#include <linux/capability.h>
#include <linux/usbdevice_fs.h>
#include <asm/byteorder.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>


#include "lpc_util.h"


/*
 * findDevice
 * --------------
 * find the device entry in /dev/bus/usb
 */
int findDevice(char *destinationDevicePathLocation, int *deviceNumber) {
    DIR *pDIR;
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












