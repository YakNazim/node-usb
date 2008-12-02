
/*
  LPCUSB, an USB device driver for LPC microcontrollers   
  Copyright (C) 2006 Bertrik Sikken (bertrik@sikken.nl)

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
  3. The name of the author may not be used to endorse or promote products
  derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, 
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/***************/
/* system libs */
/***************/
#include <string.h>                     // memcpy

/**************/
/* local libs */
/**************/

/* usb isochronous setup and utility functions */
#include "usbisoc.h"

#include "type.h"
#include "debug.h"

#ifdef LPC214x
#include "lpc214x.h"
#endif
#ifdef LPC23xx
#include "lpc23xx.h"
#endif

#include "usbisoc.h"


#include "armVIC.h"

#include "hal.h"
#include "console.h"
#include "usbapi.h"
#include "usbhw_lpc.h"

#include "serial_fifo.h"

#define DEBUG_LED_ON(x)         IOCLR0 = (1 << x);
#define DEBUG_LED_OFF(x)        IOSET0 = (1 << x);

/* serial port              */
#define BAUD_RATE               115200

/* Define endpoints for usb */
/* Interrupt                */
#define INT_OUT_EP                0x01
#define INT_IN_EP                 0x81

/* Bulk                     */
#define BULK_OUT_EP               0x02
#define BULK_IN_EP                0x82

/* Isochronous              */
#define ISOC_OUT_EP_A             0x03
#define ISOC_IN_EP_A              0x83

#define ISOC_OUT_EP_B             0x06
#define ISOC_IN_EP_B              0x86

#define ISOC_OUT_EP_C             0x0C
#define ISOC_IN_EP_C              0x8C

/* Control max packet           */
#define MAX_PACKET_SIZE_CTL       64
/* Interrupt max packet           */
#define MAX_PACKET_SIZE_INT       64
/* Bulk max packet           */
#define MAX_PACKET_SIZE_BULK      64
/* Max packet size in ISOC mode */
#define MAX_PACKET_SIZE_ISOC      1023

/* little endian word?          */
#define LE_WORD(x)              ((x)&0xFF),((x)>>8)

/* vector interupts for usb     */
#define INT_VECT_NUM            0

#define IRQ_MASK                0x00000080

static U8 abBulkBuf[64];
static U8 abClassReqData[8];
static volatile BOOL fBulkInBusy;
static volatile BOOL fChainDone;

static U8 txdata[VCOM_FIFO_SIZE];
static U8 rxdata[VCOM_FIFO_SIZE];

static fifo_t txfifo;
static fifo_t rxfifo;

// forward declaration of interrupt handler
/*static void USBIntHandler(void) __attribute__ ((interrupt(IRQ)));*/

/* 'naked' means remove prologue and epilogue from function calls  */
/* We use our own entry/exit code for interrupts                   */
static void USBIntHandler(void) __attribute__ ((interrupt(IRQ), naked));


static const U8 isoDescriptors[] = {

// device descriptor
// struct offset 0x0                 // (bytes)    Name                (notes)
    0x12,                            // (1)        bLength             (length of descriptor) 18d 
    DESC_DEVICE,                     // (1)        bDescriptorType     (0x1)
    LE_WORD(0x0101),                 // (2)        bcdUSB              (version number)
    0x00,                            // (1)        bDeviceClass        (vendor specific - each interface independent)
    0x00,                            // (1)        bDeviceSubClass     (should be 0 if bDeviceClass is 0)
    0xFF,                            // (1)        bDeviceProtocol     (spec says 0xff for vendor defined)
    MAX_PACKET_SIZE_CTL,                // (1)        bMaxPacketSize      (control endpoint specific)
    LE_WORD(0xFFFF),                 // (2)        idVendor
    LE_WORD(0x0005),                 // (2)        idProduct
    LE_WORD(0x0100),                 // (2)        bcdDevice
    0x01,                            // (1)        iManufacturer       (index of Manufacturer String Descriptor)
    0x02,                            // (1)        iProduct            (index of Product String Descriptor)
    0x03,                            // (1)        iSerialNumber       (index of Serial number String Descriptor)
    0x01,                            // (1)        bNumConfigurations  (just iso for now)

// configuration descriptor
// struct offset (0x12) (18d)        // (bytes)    Name                (notes)
    0x09,                            // (1)        bLength             (length of descriptor)
    DESC_CONFIGURATION,              // (1)        bDescriptorType     (0x2)
    LE_WORD(39),                     // (2)  (0x2E)      wTotalLength  (TotalLength(configurations?) of Data Returned)
                                     //          guess: |config descript + ep descriptors + interface descriptors |
    0x01,                            // (1)        bNumInterfaces      (ISO interface: 1 (*2* endpoints in *1* interface) 
    0x01,                            // (1)        bConfigurationValue (use this number to select THIS configuration)
    0x01,                            // (1)        iConfiguration      (index of string descriptor for THIS configuration)
    0xC0,                            // (1)        bmAttributes        (bitmap: 110----- : 
                                     //                                    bus powered, 
                                     //                                    self powered, 
                                     //                                    no remote wakeup) 
    0x32,                            // (1)        bMaxPower           (50d == 100mA 
                                     //                                   units are (2mA)==> 50d (2mA)
// iso interface
// struct offset (0x1b) (27d)        // (bytes)     Name               (notes)
    0x09,                            // (1)         bLength            (length of descriptor)
    DESC_INTERFACE,                  // (1)         bDescriptorType    (0x4)
    0x00,                            // (1)         bInterfaceNumber   (0x1)
    0x00,                            // (1)         bAlternateSetting  (0x0)
    0x0A,                            // (1)         bNumEndPoints      (excluding endpoint 0) (logical or physical?)
    0xFF,                            // (1)         bInterfaceClass    ('vendor specific' choice)
    0x01,                            // (1)         bInterfaceSubClass (xxh by spec for the InterfaceClass choice ffh)
    0x01,                            // (1)         bInterfaceProtocol (xxh by spec for the InterfaceClass choice ffh)
    0x02,                            // (1)         iInterface         (index of string descriptor for this interface)

// notification EP (interrupt)
// struct offset (0x33) (51d)        // (bytes)     Name               (notes)
    0x07,                            // (1)         bLength 
    DESC_ENDPOINT,                   // (1)         bDescriptorType    (0x5)
    INT_OUT_EP,                      // (1)         bEndpointAddress
    0x03,                            // (1)         bmAttributes       (bitmap)
                                     //             7 6 5 4 3 2 1 0    (bit index)
                                     //             x x x x x x 1 1    (0x3 ==> interrupt)      
    LE_WORD(MAX_PACKET_SIZE_INT),    // (1)         wMaxPacketSize
    0x0A,                            // (1)         bInterval          ([1..255] for interrupt)

// notification EP (interrupt)
// struct offset (0x33) (51d)        // (bytes)     Name               (notes)
    0x07,                            // (1)         bLength 
    DESC_ENDPOINT,                   // (1)         bDescriptorType    (0x5)
    INT_IN_EP,                       // (1)         bEndpointAddress
    0x03,                            // (1)         bmAttributes       (bitmap)
                                     //             7 6 5 4 3 2 1 0    (bit index)
                                     //             x x x x x x 1 1    (0x3 ==> interrupt)      
    LE_WORD(MAX_PACKET_SIZE_INT),    // (1)         wMaxPacketSize
    0x0A,                            // (1)         bInterval          ([1..255] for interrupt)


// data EP ( bulk )
// struct offset (0x33) (51d)        // (bytes)     Name               (notes)
    0x07,                            // (1)         bLength 
    DESC_ENDPOINT,                   // (1)         bDescriptorType    (0x5)
    BULK_OUT_EP,                     // (1)         bEndpointAddress
    0x04,                            // (1)         bmAttributes       (bitmap)
                                     //             7 6 5 4 3 2 1 0    (bit index)
                                     //             x x x x x x 1 0    (0x4 ==> bulk)      
    LE_WORD(MAX_PACKET_SIZE_BULK),   // (1)         wMaxPacketSize
    0x0A,                            // (1)         bInterval          ([1..255] for interrupt)

// data EP ( bulk )
// struct offset (0x33) (51d)        // (bytes)     Name               (notes)
    0x07,                            // (1)         bLength 
    DESC_ENDPOINT,                   // (1)         bDescriptorType    (0x5)
    BULK_IN_EP,                      // (1)         bEndpointAddress
    0x04,                            // (1)         bmAttributes       (bitmap)
                                     //             7 6 5 4 3 2 1 0    (bit index)
                                     //             x x x x x x 1 0    (0x4 ==> for bulk)      
    LE_WORD(MAX_PACKET_SIZE_BULK),   // (1)         wMaxPacketSize
    0x0A,                            // (1)         bInterval          ([1..255] for interrupt)

// struct offset (0x2c) (44d)        // (bytes)     Name               (notes)
    0x07,                            // (1)         bLength
    DESC_ENDPOINT,                   // (1)         bDescriptorType    (0x5)
    ISOC_OUT_EP_A,                   // (1)         bEndpointAddress   (0x03)
    0x0D,                            // (1)         bmAttributes       (bitmap)
                                     //             7 6 5 4 3 2 1 0    (bit number)
                                     //             0 0 0 0 1 1 0 1    (0xd)
    LE_WORD(MAX_PACKET_SIZE_ISOC),        // (2)         wMaxPacketSize 
    0x01,                            // (1)         bInterval          (Iso must equal 1...spec)


// struct offset (0x25) (37d)        // (bytes)     Name               (notes)
    0x07,                            // (1)         bLength
    DESC_ENDPOINT,                   // (1)         bDescriptorType    (0x5)
    ISOC_IN_EP_A,                    // (1)         bEndpointAddress   (0x83)
    0x0D,                            // (1)         bmAttributes       (bitmap)
                                     //             7 6 5 4 3 2 1 0    (bit number)
                                     //             0 0 0 0 1 1 0 1    (0xd)
    LE_WORD(MAX_PACKET_SIZE_ISOC),        // (2)         wMaxPacketSize 
    0x01,                            // (1)         bInterval          (Iso must equal 1...spec)


// struct offset (0x25) (37d)        // (bytes)     Name               (notes)
    0x07,                            // (1)         bLength
    DESC_ENDPOINT,                   // (1)         bDescriptorType    (0x5)
    ISOC_OUT_EP_B,                   // (1)         bEndpointAddress   (0x83)
    0x0D,                            // (1)         bmAttributes       (bitmap)
                                     //             7 6 5 4 3 2 1 0    (bit number)
                                     //             0 0 0 0 1 1 0 1    (0xd)
    LE_WORD(MAX_PACKET_SIZE_ISOC),        // (2)         wMaxPacketSize 
    0x01,                            // (1)         bInterval          (Iso must equal 1...spec)


// struct offset (0x2c) (44d)        // (bytes)     Name               (notes)
    0x07,                            // (1)         bLength
    DESC_ENDPOINT,                   // (1)         bDescriptorType    (0x5)
    ISOC_IN_EP_B,                    // (1)         bEndpointAddress   (0x03)
    0x0D,                            // (1)         bmAttributes       (bitmap)
                                     //             7 6 5 4 3 2 1 0    (bit number)
                                     //             0 0 0 0 1 1 0 1    (0xd)
    LE_WORD(MAX_PACKET_SIZE_ISOC),        // (2)         wMaxPacketSize 
    0x01,                            // (1)         bInterval          (Iso must equal 1...spec)


// struct offset (0x2c) (44d)        // (bytes)     Name               (notes)
    0x07,                            // (1)         bLength
    DESC_ENDPOINT,                   // (1)         bDescriptorType    (0x5)
    ISOC_OUT_EP_C,                   // (1)         bEndpointAddress   (0x03)
    0x0D,                            // (1)         bmAttributes       (bitmap)
                                     //             7 6 5 4 3 2 1 0    (bit number)
                                     //             0 0 0 0 1 1 0 1    (0xd)
    LE_WORD(MAX_PACKET_SIZE_ISOC),        // (2)         wMaxPacketSize 
    0x01,                            // (1)         bInterval          (Iso must equal 1...spec)


// struct offset (0x25) (37d)        // (bytes)     Name               (notes)
    0x07,                            // (1)         bLength
    DESC_ENDPOINT,                   // (1)         bDescriptorType    (0x5)
    ISOC_IN_EP_C,                    // (1)         bEndpointAddress   (0x83)
    0x0D,                            // (1)         bmAttributes       (bitmap)
                                     //             7 6 5 4 3 2 1 0    (bit number)
                                     //             0 0 0 0 1 1 0 1    (0xd)
    LE_WORD(MAX_PACKET_SIZE_ISOC),        // (2)         wMaxPacketSize 
    0x01,                            // (1)         bInterval          (Iso must equal 1...spec)



// string descriptors
// struct offset (0x39)     (57d)
    0x04,                            // (1)         bLength    
    DESC_STRING,                     // (1)         bDescriptorType   (String Descriptor 0x3)
    LE_WORD(0x0409),                 // (2)         wLANGID[0]        (0x0409 English-US)

    0x0E,                            // (1)         bLength           (index 01) 
    DESC_STRING,                     // (1)         bDescriptorType   (0x3)
    // unicode encoded string 
    'L', 0, 'P', 0, 'C', 0, 'U', 0, 'S', 0, 'B', 0,  

    0x14,                            // (1)         bLength           (index 02)
    DESC_STRING,                     // (1)         bDescriptorType   (0x3)
    'U', 0, 'S', 0, 'B', 0, 'I', 0, 's', 0, 'o', 0, 'c', 0, 'h', 0, 'r', 0,

    0x12,                            // (1)         bLength           (index 03)
    DESC_STRING,                     // (1)         bDescriptorType   (0x3)
    'D', 0, 'E', 0, 'A', 0, 'D', 0, 'C', 0, '0', 0, 'D', 0, 'E', 0,

    0x16,                            // (1)         bLength           (index 04)
    DESC_STRING,                     // (1)         bDescriptorType   (0x3)
    'P', 0, 'S', 0, 'A', 0, 'S', 0, 'U', 0, 'S', 0, 'B', 0, 'I', 0, 's', 0, 'o', 0,

// terminating zero
    0
};



U8 isConnected = 0;

/**
   Local function to handle the USB-CDC class requests
                
   @param [in] pSetup
   @param [out] piLen
   @param [out] ppbData
*/


/**
   Initialises the VCOM port.
   Call this function before using VCOM_putchar or VCOM_getchar
*/
/* void VCOM_init(void) */
/* { */
/*     fifo_init(&txfifo, txdata); */
/*     fifo_init(&rxfifo, rxdata); */
/*     fBulkInBusy = FALSE; */
/*     fChainDone = TRUE; */
/* } */


/**
   Writes one character to VCOM port
        
   @param [in] c character to write
   @returns character written, or EOF if character could not be written
*/

/* int VCOM_putchar(int c) */
/* { */
/*     return fifo_put(&txfifo, c) ? c : EOF; */
/* } */



/**
   Reads one character from VCOM port
        
   @returns character read, or EOF if character could not be read
*/

/* int VCOM_getchar(void) */
/* { */
/*     U8 c; */
        
/*     return fifo_get(&rxfifo, &c) ? c : EOF; */
/* } */




/******************************************************************************
 *
 * MACRO Name: ISR_ENTRY()
 *
 * Description:
 *    This MACRO is used upon entry to an ISR.  The current version of
 *    the gcc compiler for ARM does not produce correct code for
 *    interrupt routines to operate properly with THUMB code.  The MACRO
 *    performs the following steps:
 *
 *    1 - Adjust address at which execution should resume after servicing
 *        ISR to compensate for IRQ entry
 *    2 - Save the non-banked registers r0-r12 and lr onto the IRQ stack.
 *    3 - Get the status of the interrupted program is in SPSR.
 *    4 - Push it onto the IRQ stack as well.
 *
 *****************************************************************************/
#define ISR_ENTRY() asm volatile(" sub   lr, lr,#4\n" \
                                 " stmfd sp!,{r0-r8,lr}\n" \
                                 " mrs   r1, spsr\n" \
                                 " stmfd sp!,{r1}")


/******************************************************************************
 *
 * MACRO Name: ISR_EXIT()
 *
 * Description:
 *    This MACRO is used to exit an ISR.  The current version of the gcc
 *    compiler for ARM does not produce correct code for interrupt
 *    routines to operate properly with THUMB code.  The MACRO performs
 *    the following steps:
 *
 *    1 - Recover SPSR value from stack       
 *    2 - and restore  its value                   
 *    3 - Pop the return address & the saved general registers from
 *        the IRQ stack & return
 *
 *****************************************************************************/
#define ISR_EXIT()  asm volatile(" ldmfd sp!,{r1}\n" \
                                 " msr   spsr_c,r1\n" \
                                 " ldmfd sp!,{r0-r8,pc}^")




/**
   Interrupt handler
        
   Simply calls the USB ISR, then signals end of interrupt to VIC
*/
void USBIntHandler(void) 
{
    ISR_ENTRY(); 
    //DBG("Z");
    USBHwISR();
    //DBG("z");
    VICVectAddr = 0x00;    // dummy write to VIC to signal end of ISR
    ISR_EXIT();
}

char hexch(const unsigned char x) {
    if( x < 10 ) {
	return('0' + x);
    } else if( x < 16 ) {
	return('A' + (x-10));
    } else {
	return('?');
    }
}



static void SendIsocIn()
{
    U8 tempBuff[1];
    tempBuff[0] = 'q';
        
    // send over USB
    USBHwEPWrite(ISOC_IN_EP, tempBuff, 1);
}

/**
   USB frame interrupt handler
        
   Called every milisecond by the hardware driver.
        
   This function is responsible for sending the first of a chain of packets
   to the host. A chain is always terminated by a short packet, either a
   packet shorter than the maximum packet size or a zero-length packet
   (as required by the windows usbser.sys driver).

*/

char tempBuff[MAX_PACKET_SIZE + 1];
static void USBFrameHandler(U16 wFrame)
{
    /*
      int i;
      int iLen = USBHwEPReadDave(ISOC_OUT_EP, tempBuff, sizeof(tempBuff));
      if (iLen > 0) {
      for(i = 0; i <iLen; i++ ) {
      DBG("%c", tempBuff[i]);
      }
      //DBG("Q!");
      }
        
    */
    //SendIsocIn();
    //if( ( (dd3 >> 1) & 0x0F ) == 0 ) {
    //enableDMAForEndpoint(ISOC_IN_EP);
    //}

        
}


/**
   USB device status handler
        
   Resets state machine when a USB reset is received.
*/
static void USBDevIntHandler(U8 bDevStatus)
{
    if ((bDevStatus & DEV_STATUS_RESET) != 0) {
	fBulkInBusy = FALSE;
    }
}

char toHex(int x) {
    if( x <= 9 ) {
	return('0' + x);
    } else if( x <= 15 ) {
	return('A' + x);
    } else {
	return('?');
    }
}



#define EP2IDX(bEP) ((((bEP)&0xF)<<1)|(((bEP)&0x80)>>7))

#define NUM_ISOC_FRAMES 4
#define BYTES_PER_ISOC_FRAME 1023
#define NUM_DMA_DESCRIPTORS 6
#define ISOC_DATA_BUFFER_SIZE (1024*6)

__attribute__ ((section (".usbdma"), aligned(128))) volatile U32* udcaHeadArray[32];
__attribute__ ((section (".usbdma"), aligned(128))) U32 isocFrameArray[NUM_ISOC_FRAMES];
__attribute__ ((section (".usbdma"), aligned(128))) volatile U32 dmaDescriptorArray[NUM_DMA_DESCRIPTORS][5];
__attribute__ ((section (".usbdma"), aligned(128))) U8 isocDataBuffer[ISOC_DATA_BUFFER_SIZE];

U16 isocFrameNumber = 1;

void magicDMA(void) {
    int i;
    //allocate source data usb ram
        
    //populate source datw with all 'A's
    for(i = 0; i < ISOC_DATA_BUFFER_SIZE; i++ ) {
	isocDataBuffer[i] = 'A';
    }
        
    USBInitializeISOCFrameArray(isocFrameArray, NUM_ISOC_FRAMES, isocFrameNumber, BYTES_PER_ISOC_FRAME);
    isocFrameNumber += NUM_ISOC_FRAMES;
        
    for(i = 0; i < NUM_DMA_DESCRIPTORS - 1; i++ ) {
	USBSetupDMADescriptor(dmaDescriptorArray[i], dmaDescriptorArray[(i+1)], 1, MAX_PACKET_SIZE, NUM_ISOC_FRAMES, isocDataBuffer, isocFrameArray );  
    }
    USBSetupDMADescriptor(dmaDescriptorArray[i], dmaDescriptorArray[(i+1)], 1, MAX_PACKET_SIZE, NUM_ISOC_FRAMES, isocDataBuffer, isocFrameArray );
        
    //Set UDCA head register to point to start of usb ram
    USBInitializeUSBDMA(udcaHeadArray);

        
    //set DDP pointer for endpoint so it knows where first DD is located, manual section 13.1
    //set index of isoc DDP to point to start DD
    udcaHeadArray[EP2IDX(ISOC_IN_EP)] = dmaDescriptorArray[0];
        
        
    DBG("First dma log output\n");
    logdd();
        
    //enable dma for endpoint
    USBEnableDMAForEndpoint(ISOC_IN_EP);
        
    //disable by setting the corresponding bit in the 'Endpoint Interrupt Enable' register to 0
        
}

void logdd(void) {
    int i;
    DBG     ("---------------------------------\n");
    /*
      DBG("udcaHeadAddr = 0x%X\n", udcaHeadAddr);

      for (i = 0; i < 32; i++) {
      DBG("udcaHeadAddr[%d] = 0x%X\n", i, udcaHeadAddr[i]);
      }
    */ 
        
    U32 dd3 = dmaDescriptorArray[0][3];
        
    DBG("dmaDescriptorArray[0] = 0x%X\n", dmaDescriptorArray[0]);
    DBG("dmaDescriptorArray[1] = 0x%X\n", dmaDescriptorArray[1]);
    DBG("dmaDescriptorArray[0][0] = 0x%X\n", dmaDescriptorArray[0][0]);
    DBG("dmaDescriptorArray[0][1] = 0x%X\n", dmaDescriptorArray[0][1]);
    DBG("dmaDescriptorArray[0][2] = 0x%X\n", dmaDescriptorArray[0][2]);
    DBG("dmaDescriptorArray[0][3] = 0x%X\n", dmaDescriptorArray[0][3]);
    DBG("dmaDescriptorArray[0][4] = 0x%X\n", dmaDescriptorArray[0][4]);
        
    if( dd3 & 0x01 ) {
	DBG("Retired\n");
    } else {
	DBG("Not Retired\n");
    }
        
    switch( (dd3 >> 1) & 0x0F ) {
        case 0:
	    DBG("Not serviced\n");
	    break;
        case 1:
	    DBG("Being serviced\n");
	    break;
        case 2:
	    DBG("Normal completion\n");
	    break;
        case 3:
	    DBG("data underrune\n");
	    break;
        case 8:
	    DBG("data overrun \n");
	    break;
        case 9:
	    DBG("system error\n");
	    break;
    }
        
    DBG("Present dma count %d\n", (dd3 >> 16));
        
    DBG("isocFrameArray = 0x%X\n", isocFrameArray);
    DBG("isocFrameArray[0] = 0x%X\n", isocFrameArray[0]);
    DBG("isocFrameArray[1] = 0x%X\n", isocFrameArray[1]);
    DBG("isocFrameArray[2] = 0x%X\n", isocFrameArray[2]);
    DBG("isocFrameArray[3] = 0x%X\n", isocFrameArray[3]);
    DBG("isocFrameArray[4] = 0x%X\n", isocFrameArray[4]);

        
        
}

/*************************************************************************
        main
        ====
**************************************************************************/
int main(void)
{
    int c, i;
        
    // PLL and MAM
    HalSysInit();

#ifdef LPC214x
    // init DBG
    ConsoleInit(60000000 / (16 * BAUD_RATE));
#else
    // init DBG
    ConsoleInit(72000000 / (16 * BAUD_RATE));
#endif

    DBG("Initialising USB stack\n");

    // initialise stack
    USBInit();

    // register descriptors
    USBRegisterDescriptors(abDescriptors);

    // register class request handler
    USBRegisterRequestHandler(REQTYPE_TYPE_CLASS, HandleClassRequest, abClassReqData);

    // register endpoint handlers
    USBHwRegisterEPIntHandler(INT_IN_EP, NULL);
        
    //USBHwRegisterEPIntHandler(ISOC_OUT_EP, IsocOut);
        
        
        
    // register frame handler
    //USBHwRegisterFrameHandler(USBFrameHandler);
        
    // register device event handler
    USBHwRegisterDevIntHandler(USBDevIntHandler);

    // initialise VCOM
//    VCOM_init();

        
    DBG("Starting USB communication\n");

#ifdef LPC214x
    (*(&VICVectCntl0+INT_VECT_NUM)) = 0x20 | 22; // choose highest priority ISR slot        
    (*(&VICVectAddr0+INT_VECT_NUM)) = (int)USBIntHandler;
#else
    VICVectCntl22 = 0x01;
    VICVectAddr22 = (int)USBIntHandler;
#endif
  
    // set up USB interrupt
    VICIntSelect &= ~(1<<22);               // select IRQ for USB
    VICIntEnable |= (1<<22);
        
    enableIRQ();

    magicDMA();
        
    // connect to bus
    USBHwConnect(TRUE);
        
    int x = 0;
    int ch  ='a';
    c = EOF;
        
    //logdd();
        
    // echo any character received (do USB stuff in interrupt)
    while (1) {

                        
	//DBG("srcBuff[1] = 0x%X\n", srcBuff[1]);

	if ( ((dmaDescriptorArray[NUM_DMA_DESCRIPTORS-1][3] >> 1) & 0x0F ) == 2 ) {
	    //normal completion
                        
	    USBDisableDMAForEndpoint(ISOC_IN_EP);
                        
	    USBInitializeISOCFrameArray(isocFrameArray, NUM_ISOC_FRAMES, isocFrameNumber, BYTES_PER_ISOC_FRAME);
	    isocFrameNumber += NUM_ISOC_FRAMES;
                
	    for (i = 0; i < NUM_DMA_DESCRIPTORS - 1; i++) {
		USBSetupDMADescriptor(dmaDescriptorArray[i], dmaDescriptorArray[(i+1)], 1, MAX_PACKET_SIZE, NUM_ISOC_FRAMES, isocDataBuffer, isocFrameArray);
	    }
	    USBSetupDMADescriptor(dmaDescriptorArray[i], dmaDescriptorArray[(i+1)], 1, MAX_PACKET_SIZE, NUM_ISOC_FRAMES, isocDataBuffer, isocFrameArray);

	    udcaHeadArray[EP2IDX(ISOC_IN_EP)] = dmaDescriptorArray[0];
                        
	    USBEnableDMAForEndpoint(ISOC_IN_EP);
	}

	// Turn light on and off.
                
	x++;
	if (x == 400000) {
                        
	    IOSET0 = (1<<11);
	    //turn on led
                        
	    if( ch > 'z' ) {
		ch = 'a';
	    }
	    //VCOM_putchar(ch);
	    //DBG("%c", ch);
	    //DBG("\r\n");
	    ch++;
                    
	    logdd();
	} else if (x >= 800000) {
	    IOCLR0 = (1<<11);
	    //turn off led
	    x = 0;
	}

    }

    return 0;
}

