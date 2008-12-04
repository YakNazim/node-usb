
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


/*
 * (2^(binterval-1))*F 
 * F= 1mS for Full speed
 * F= 125uS for High speed
 */
#define ISOC_BINTERVAL            0x01

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

#define EP2IDX(bEP) ((((bEP)&0xF)<<1)|(((bEP)&0x80)>>7))

#define NUM_ISOC_FRAMES 4
#define BYTES_PER_ISOC_FRAME 1023
#define NUM_DMA_DESCRIPTORS 6
#define ISOC_DATA_BUFFER_SIZE (1024*6)

__attribute__ ((section (".usbdma"), aligned(128))) volatile U32* udcaHeadArray[32];
__attribute__ ((section (".usbdma"), aligned(128))) U32 isocFrameArray[NUM_ISOC_FRAMES];
__attribute__ ((section (".usbdma"), aligned(128))) volatile U32 dmaDescriptorArray[NUM_DMA_DESCRIPTORS][5];
__attribute__ ((section (".usbdma"), aligned(128))) U8 isocDataBuffer[ISOC_DATA_BUFFER_SIZE];


// bulk data fifos.

static fifo_t txfifo;
static fifo_t rxfifo;

static volatile BOOL fBulkInBusy;

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
    0x04,                            // (1)        iSerialNumber       (index of Serial number String Descriptor)
    0x01,                            // (1)        bNumConfigurations  (just iso for now)
///////////////////////////////////
// configuration descriptor
///////////////////////////////////
// struct offset (0x12) (18d)        // (bytes)    Name                (notes)
    0x09,                            // (1)        bLength             (length of descriptor)
    DESC_CONFIGURATION,              // (1)        bDescriptorType     (0x2)
    LE_WORD(88),                     // (2)  (0x2E)      wTotalLength  (TotalLength(configurations?) of Data Returned)
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

///////////////////////////////////
// interface descriptor
///////////////////////////////////
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

/////////////////////////////////
// notification EP OUT (interrupt)
/////////////////////////////////
// struct offset (0x33) (51d)        // (bytes)     Name               (notes)
    0x07,                            // (1)         bLength 
    DESC_ENDPOINT,                   // (1)         bDescriptorType    (0x5)
    INT_OUT_EP,                      // (1)         bEndpointAddress
    0x03,                            // (1)         bmAttributes       (bitmap)
                                     //             7 6 5 4 3 2 1 0    (bit index)
                                     //             x x x x x x 1 1    (0x3 ==> interrupt)      
    LE_WORD(MAX_PACKET_SIZE_INT),    // (1)         wMaxPacketSize
    0x0A,                            // (1)         bInterval          ([1..255] for interrupt)

/////////////////////////////////
// notification EP IN (interrupt)
/////////////////////////////////
// struct offset (0x33) (51d)        // (bytes)     Name               (notes)
    0x07,                            // (1)         bLength 
    DESC_ENDPOINT,                   // (1)         bDescriptorType    (0x5)
    INT_IN_EP,                       // (1)         bEndpointAddress
    0x03,                            // (1)         bmAttributes       (bitmap)
                                     //             7 6 5 4 3 2 1 0    (bit index)
                                     //             x x x x x x 1 1    (0x3 ==> interrupt)      
    LE_WORD(MAX_PACKET_SIZE_INT),    // (1)         wMaxPacketSize
    0x0A,                            // (1)         bInterval          ([1..255] for interrupt)

////////////////////////////////
// data EP OUT ( bulk )
////////////////////////////////
// struct offset (0x33) (51d)        // (bytes)     Name               (notes)
    0x07,                            // (1)         bLength 
    DESC_ENDPOINT,                   // (1)         bDescriptorType    (0x5)
    BULK_OUT_EP,                     // (1)         bEndpointAddress
    0x02,                            // (1)         bmAttributes       (bitmap)
                                     //             7 6 5 4 3 2 1 0    (bit index)
                                     //             x x x x x 0 2 0    (0x2 ==> bulk)      
    LE_WORD(MAX_PACKET_SIZE_BULK),   // (1)         wMaxPacketSize
    0x0A,                            // (1)         bInterval          ([1..255] for interrupt)

/////////////////////////////////
// data EP IN ( bulk )
/////////////////////////////////
// struct offset (0x33) (51d)        // (bytes)     Name               (notes)
    0x07,                            // (1)         bLength 
    DESC_ENDPOINT,                   // (1)         bDescriptorType    (0x5)
    BULK_IN_EP,                      // (1)         bEndpointAddress
    0x02,                            // (1)         bmAttributes       (bitmap)
                                     //             7 6 5 4 3 2 1 0    (bit index)
                                     //             x x x x x 0 2 0    (0x2 ==> for bulk)      
    LE_WORD(MAX_PACKET_SIZE_BULK),   // (1)         wMaxPacketSize
    0x0A,                            // (1)         bInterval          ([1..255] for interrupt)

/////////////////////////////////
// isoc EP OUT
/////////////////////////////////
// struct offset (0x2c) (44d)        // (bytes)     Name               (notes)
    0x07,                            // (1)         bLength
    DESC_ENDPOINT,                   // (1)         bDescriptorType    (0x5)
    ISOC_OUT_EP_A,                   // (1)         bEndpointAddress   (0x03)
    0x0D,                            // (1)         bmAttributes       (bitmap)
                                     //             7 6 5 4 3 2 1 0    (bit number)
                                     //             0 0 0 0 1 1 0 1    (0xd)
    LE_WORD(MAX_PACKET_SIZE_ISOC),   // (2)         wMaxPacketSize 
    ISOC_BINTERVAL,                  // (1)         bInterval          (Iso must equal 1...spec)

////////////////////////////////
// isoc EP IN
////////////////////////////////
// struct offset (0x25) (37d)        // (bytes)     Name               (notes)
    0x07,                            // (1)         bLength
    DESC_ENDPOINT,                   // (1)         bDescriptorType    (0x5)
    ISOC_IN_EP_A,                    // (1)         bEndpointAddress   (0x83)
    0x0D,                            // (1)         bmAttributes       (bitmap)
                                     //             7 6 5 4 3 2 1 0    (bit number)
                                     //             0 0 0 0 1 1 0 1    (0xd)
    LE_WORD(MAX_PACKET_SIZE_ISOC),   // (2)         wMaxPacketSize
    ISOC_BINTERVAL,                  // (1)         bInterval          (Iso must equal 1...spec)


/////////////////////////////////
// isoc EP OUT
/////////////////////////////////
// struct offset (0x25) (37d)        // (bytes)     Name               (notes)
    0x07,                            // (1)         bLength
    DESC_ENDPOINT,                   // (1)         bDescriptorType    (0x5)
    ISOC_OUT_EP_B,                   // (1)         bEndpointAddress   (0x83)
    0x0D,                            // (1)         bmAttributes       (bitmap)
                                     //             7 6 5 4 3 2 1 0    (bit number)
                                     //             0 0 0 0 1 1 0 1    (0xd)
    LE_WORD(MAX_PACKET_SIZE_ISOC),   // (2)         wMaxPacketSize
    ISOC_BINTERVAL,                            // (1)         bInterval          (Iso must equal 1...spec)

/////////////////////////////////
// isoc EP IN
/////////////////////////////////
// struct offset (0x2c) (44d)        // (bytes)     Name               (notes)
    0x07,                            // (1)         bLength
    DESC_ENDPOINT,                   // (1)         bDescriptorType    (0x5)
    ISOC_IN_EP_B,                    // (1)         bEndpointAddress   (0x03)
    0x0D,                            // (1)         bmAttributes       (bitmap)
                                     //             7 6 5 4 3 2 1 0    (bit number)
                                     //             0 0 0 0 1 1 0 1    (0xd)
    LE_WORD(MAX_PACKET_SIZE_ISOC),   // (2)         wMaxPacketSize
    ISOC_BINTERVAL,                            // (1)         bInterval          (Iso must equal 1...spec)


/////////////////////////////////
// isoc EP OUT
/////////////////////////////////
// struct offset (0x2c) (44d)        // (bytes)     Name               (notes)
    0x07,                            // (1)         bLength
    DESC_ENDPOINT,                   // (1)         bDescriptorType    (0x5)
    ISOC_OUT_EP_C,                   // (1)         bEndpointAddress   (0x03)
    0x0D,                            // (1)         bmAttributes       (bitmap)
                                     //             7 6 5 4 3 2 1 0    (bit number)
                                     //             0 0 0 0 1 1 0 1    (0xd)
    LE_WORD(MAX_PACKET_SIZE_ISOC),   // (2)         wMaxPacketSize
    ISOC_BINTERVAL,                            // (1)         bInterval          (Iso must equal 1...spec)


/////////////////////////////////
// isoc EP IN
/////////////////////////////////
// struct offset (0x25) (37d)        // (bytes)     Name               (notes)
    0x07,                            // (1)         bLength
    DESC_ENDPOINT,                   // (1)         bDescriptorType    (0x5)
    ISOC_IN_EP_C,                    // (1)         bEndpointAddress   (0x83)
    0x0D,                            // (1)         bmAttributes       (bitmap)
                                     //             7 6 5 4 3 2 1 0    (bit number)
                                     //             0 0 0 0 1 1 0 1    (0xd)
    LE_WORD(MAX_PACKET_SIZE_ISOC),   // (2)         wMaxPacketSize
    ISOC_BINTERVAL,                            // (1)         bInterval          (Iso must equal 1...spec)


/////////////////////////////////
// string descriptors
/////////////////////////////////
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

/**
	Local function to handle incoming bulk data
		
	@param [in] bEP
	@param [in] bEPStatus
 */
static void BulkOut(U8 bEP, U8 bEPStatus)
{
	int i, iLen;

	if (fifo_free(&rxfifo) < MAX_PACKET_SIZE) {
		// may not fit into fifo
		return;
	}

	// get data from USB into intermediate buffer
	iLen = USBHwEPRead(bEP, abBulkBuf, sizeof(abBulkBuf));
	for (i = 0; i < iLen; i++) {
		// put into FIFO
		if (!fifo_put(&rxfifo, abBulkBuf[i])) {
			// overflow... :(
			ASSERT(FALSE);
			break;
		}
	}
}



/**
	Sends the next packet in chain of packets to the host
		
	@param [in] bEP
	@param [in] bEPStatus
 */
static void SendNextBulkIn(U8 bEP, BOOL fFirstPacket)
{
	int iLen;

	// this transfer is done
	fBulkInBusy = FALSE;
	
	// first packet?
	if (fFirstPacket) {
		fChainDone = FALSE;
	}

	// last packet?
	if (fChainDone) {
		return;
	}
	
	// get up to MAX_PACKET_SIZE bytes from transmit FIFO into intermediate buffer
	for (iLen = 0; iLen < MAX_PACKET_SIZE; iLen++) {
		if (!fifo_get(&txfifo, &abBulkBuf[iLen])) {
			break;
		}
	}
	
	// send over USB
	USBHwEPWrite(bEP, abBulkBuf, iLen);
	fBulkInBusy = TRUE;

	// was this a short packet?
	if (iLen < MAX_PACKET_SIZE) {
		fChainDone = TRUE;
	}
}


/**
	Local function to handle outgoing bulk data
		
	@param [in] bEP
	@param [in] bEPStatus
 */
static void BulkIn(U8 bEP, U8 bEPStatus)
{
	SendNextBulkIn(bEP, FALSE);
}



// print out hex char
char hexch(const unsigned char x) {
    if( x < 10 ) {
        return('0' + x);
    } else if( x < 16 ) {
        return('A' + (x-10));
    } else {
        return('?');
    }
}

/**
   USB frame interrupt handler
        
   Called every milisecond by the hardware driver.
        
   This function is responsible for sending the first of a chain of packets
   to the host. A chain is always terminated by a short packet, either a
   packet shorter than the maximum packet size or a zero-length packet
   (as required by the windows usbser.sys driver).

*/

/*
 *	Initialises the bulk port.
 */
void bulk_init(void)
{
	fifo_init(&txfifo, txdata);
	fifo_init(&rxfifo, rxdata);
	fBulkInBusy = FALSE;
	fChainDone = TRUE;
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

/*************************************************************************
        main
        ====
**************************************************************************/
int main(void)
{

    ///////////////////////////////////////////////////
    // PRELUDE
    ///////////////////////////////////////////////////
        
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
    USBRegisterDescriptors(isoDescriptors);

    // register endpoint handlers
    USBHwRegisterEPIntHandler(INT_IN_EP, NULL);
    USBHwRegisterEPIntHandler(BULK_IN_EP, BulkIn);
    USBHwRegisterEPIntHandler(BULK_OUT_EP, BulkOut);

    // USBHwRegisterEPIntHandler(ISOC_OUT_EP, IsocOut);
                
    // register frame handler
    // USBHwRegisterFrameHandler(USBFrameHandler);
        
    // register device event handler
    USBHwRegisterDevIntHandler(USBDevIntHandler);

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
        
    // connect to bus
    USBHwConnect(TRUE);


    /////////////////////////////////////////////
    // FUNCTION
    /////////////////////////////////////////////

    // int

    // bulk   - echo
    bulk_init();
    // isoc A - Broadcast numbers

    // isoc B - echo

    // isoc C
    








    //////////////////////////////////////////
    // DEBUG LIGHTS
    //////////////////////////////////////////

    int c;
    int x = 0;
    int ch  ='a';
    c = EOF;
        
    // echo any character received (do USB stuff in interrupt)
    while (1) {

            
        // Turn light on and off.
                
        x++;
        if (x == 400000) {
                        
            IOSET0 = (1<<11);
            //turn on led
                        
            if( ch > 'z' ) {
                ch = 'a';
            }

            ch++;
                    
        } else if (x >= 800000) {
            IOCLR0 = (1<<11);
            //turn off led
            x = 0;
        }

    }

    return 0;
}


/* Bus 002 Device 046: ID ffff:0005   */
/* Device Descriptor: */
/*   bLength                18 */
/*   bDescriptorType         1 */
/*   bcdUSB               1.01 */
/*   bDeviceClass            0 (Defined at Interface level) */
/*   bDeviceSubClass         0  */
/*   bDeviceProtocol       255  */
/*   bMaxPacketSize0        64 */
/*   idVendor           0xffff  */
/*   idProduct          0x0005  */
/*   bcdDevice            1.00 */
/*   iManufacturer           1 LPCUSB */
/*   iProduct                2 USBIsochr */
/*   iSerial                 4 PSASUSBIso */
/*   bNumConfigurations      1 */
/*   Configuration Descriptor: */
/*     bLength                 9 */
/*     bDescriptorType         2 */
/*     wTotalLength           88 */
/*     bNumInterfaces          1 */
/*     bConfigurationValue     1 */
/*     iConfiguration          1 LPCUSB */
/*     bmAttributes         0xc0 */
/*       Self Powered */
/*     MaxPower              100mA */
/*     Interface Descriptor: */
/*       bLength                 9 */
/*       bDescriptorType         4 */
/*       bInterfaceNumber        0 */
/*       bAlternateSetting       0 */
/*       bNumEndpoints          10 */
/*       bInterfaceClass       255 Vendor Specific Class */
/*       bInterfaceSubClass      1  */
/*       bInterfaceProtocol      1  */
/*       iInterface              2 USBIsochr */
/*       Endpoint Descriptor: */
/*         bLength                 7 */
/*         bDescriptorType         5 */
/*         bEndpointAddress     0x01  EP 1 OUT */
/*         bmAttributes            3 */
/*           Transfer Type            Interrupt */
/*           Synch Type               None */
/*           Usage Type               Data */
/*         wMaxPacketSize     0x0040  1x 64 bytes */
/*         bInterval              10 */
/*       Endpoint Descriptor: */
/*         bLength                 7 */
/*         bDescriptorType         5 */
/*         bEndpointAddress     0x81  EP 1 IN */
/*         bmAttributes            3 */
/*           Transfer Type            Interrupt */
/*           Synch Type               None */
/*           Usage Type               Data */
/*         wMaxPacketSize     0x0040  1x 64 bytes */
/*         bInterval              10 */
/*       Endpoint Descriptor: */
/*         bLength                 7 */
/*         bDescriptorType         5 */
/*         bEndpointAddress     0x02  EP 2 OUT */
/*         bmAttributes            2 */
/*           Transfer Type            Bulk */
/*           Synch Type               None */
/*           Usage Type               Data */
/*         wMaxPacketSize     0x0040  1x 64 bytes */
/*         bInterval              10 */
/*       Endpoint Descriptor: */
/*         bLength                 7 */
/*         bDescriptorType         5 */
/*         bEndpointAddress     0x82  EP 2 IN */
/*         bmAttributes            2 */
/*           Transfer Type            Bulk */
/*           Synch Type               None */
/*           Usage Type               Data */
/*         wMaxPacketSize     0x0040  1x 64 bytes */
/*         bInterval              10 */
/*       Endpoint Descriptor: */
/*         bLength                 7 */
/*         bDescriptorType         5 */
/*         bEndpointAddress     0x03  EP 3 OUT */
/*         bmAttributes           13 */
/*           Transfer Type            Isochronous */
/*           Synch Type               Synchronous */
/*           Usage Type               Data */
/*         wMaxPacketSize     0x03ff  1x 1023 bytes */
/*         bInterval               1 */
/*       Endpoint Descriptor: */
/*         bLength                 7 */
/*         bDescriptorType         5 */
/*         bEndpointAddress     0x83  EP 3 IN */
/*         bmAttributes           13 */
/*           Transfer Type            Isochronous */
/*           Synch Type               Synchronous */
/*           Usage Type               Data */
/*         wMaxPacketSize     0x03ff  1x 1023 bytes */
/*         bInterval               1 */
/*       Endpoint Descriptor: */
/*         bLength                 7 */
/*         bDescriptorType         5 */
/*         bEndpointAddress     0x06  EP 6 OUT */
/*         bmAttributes           13 */
/*           Transfer Type            Isochronous */
/*           Synch Type               Synchronous */
/*           Usage Type               Data */
/*         wMaxPacketSize     0x03ff  1x 1023 bytes */
/*         bInterval               1 */
/*       Endpoint Descriptor: */
/*         bLength                 7 */
/*         bDescriptorType         5 */
/*         bEndpointAddress     0x86  EP 6 IN */
/*         bmAttributes           13 */
/*           Transfer Type            Isochronous */
/*           Synch Type               Synchronous */
/*           Usage Type               Data */
/*         wMaxPacketSize     0x03ff  1x 1023 bytes */
/*         bInterval               1 */
/*       Endpoint Descriptor: */
/*         bLength                 7 */
/*         bDescriptorType         5 */
/*         bEndpointAddress     0x0c  EP 12 OUT */
/*         bmAttributes           13 */
/*           Transfer Type            Isochronous */
/*           Synch Type               Synchronous */
/*           Usage Type               Data */
/*         wMaxPacketSize     0x03ff  1x 1023 bytes */
/*         bInterval               1 */
/*       Endpoint Descriptor: */
/*         bLength                 7 */
/*         bDescriptorType         5 */
/*         bEndpointAddress     0x8c  EP 12 IN */
/*         bmAttributes           13 */
/*           Transfer Type            Isochronous */
/*           Synch Type               Synchronous */
/*           Usage Type               Data */
/*         wMaxPacketSize     0x03ff  1x 1023 bytes */
/*         bInterval               1 */
/* Device Status:     0x0000 */
/*   (Bus Powered) */
