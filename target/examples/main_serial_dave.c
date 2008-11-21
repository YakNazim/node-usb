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

/*
	Minimal implementation of a USB serial port, using the CDC class.
	This example application simply echoes everything it receives right back
	to the host.

	Windows:
	Extract the usbser.sys file from .cab file in C:\WINDOWS\Driver Cache\i386
	and store it somewhere (C:\temp is a good place) along with the usbser.inf
	file. Then plug in the LPC214x and direct windows to the usbser driver.
	Windows then creates an extra COMx port that you can open in a terminal
	program, like hyperterminal.

	Linux:
	The device should be recognised automatically by the cdc_acm driver,
	which creates a /dev/ttyACMx device file that acts just like a regular
	serial port.

*/


#include <string.h>			// memcpy

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


#define DEBUG_LED_ON(x)     IOCLR0 = (1 << x);
#define DEBUG_LED_OFF(x)    IOSET0 = (1 << x);


#define BAUD_RATE	115200

#define INT_IN_EP		0x81

#define ISOC_OUT_EP     0x06
#define ISOC_IN_EP      0x83


#define MAX_PACKET_SIZE	1023

#define LE_WORD(x)		((x)&0xFF),((x)>>8)

// CDC definitions
#define CS_INTERFACE			0x24
#define CS_ENDPOINT				0x25

#define	SET_LINE_CODING			0x20
#define	GET_LINE_CODING			0x21
#define	SET_CONTROL_LINE_STATE	0x22

#define	INT_VECT_NUM	0

#define IRQ_MASK 0x00000080

// data structure for GET_LINE_CODING / SET_LINE_CODING class requests
typedef struct {
	U32		dwDTERate;
	U8		bCharFormat;
	U8		bParityType;
	U8		bDataBits;
} TLineCoding;

static TLineCoding LineCoding = {115200, 0, 0, 8};
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
static void USBIntHandler(void) __attribute__ ((interrupt(IRQ), naked));
/*__attribute__ ((interrupt("IRQ")));*/


static const U8 abDescriptors[] = {

// device descriptor
	0x12,
	DESC_DEVICE,
	LE_WORD(0x0101),			// bcdUSB
	0x02,						// bDeviceClass
	0x00,						// bDeviceSubClass
	0x00,						// bDeviceProtocol
	MAX_PACKET_SIZE0,			// bMaxPacketSize
	LE_WORD(0xFFFF),			// idVendor
	LE_WORD(0x0005),			// idProduct
	LE_WORD(0x0100),			// bcdDevice
	0x01,						// iManufacturer
	0x02,						// iProduct
	0x03,						// iSerialNumber
	0x01,						// bNumConfigurations

	
	
// configuration descriptor
	0x09,
	DESC_CONFIGURATION,
	LE_WORD(32), //sizeof(this configuration descriptor) + sizeof(all interfaces defined)   //LE_WORD(67),				// wTotalLength
	0x01, //0x02,				// bNumInterfaces
	0x01,						// bConfigurationValue
	0x00,						// iConfiguration
	0xC0,						// bmAttributes
	0x32,						// bMaxPower
	
	
// data class interface descriptor   9+7+7=23
	0x09,
	DESC_INTERFACE,
	0x00,						// bInterfaceNumber
	0x00,						// bAlternateSetting
	0x02,//DC				    // bNumEndPoints
	0xFF,// 0x0A,				// bInterfaceClass = data
	0x00,						// bInterfaceSubClass
	0x00,						// bInterfaceProtocol
	0x00,						// iInterface
	
// data EP OUT
	0x07,
	DESC_ENDPOINT,
	ISOC_OUT_EP,				// bEndpointAddress
	0x0D,					    // bmAttributes = isoc, syncronous, data endpoint
	LE_WORD(MAX_PACKET_SIZE),	// wMaxPacketSize
	0x09,						// bInterval	
	
	// data EP OUT
	0x07,
	DESC_ENDPOINT,
	ISOC_IN_EP,				    // bEndpointAddress
	0x0D,					    // bmAttributes = isoc, syncronous, data endpoint
	LE_WORD(MAX_PACKET_SIZE),	// wMaxPacketSize
	0x01,						// bInterval	
	
	// string descriptors
	0x04,
	DESC_STRING,
	LE_WORD(0x0409),

	0x0E,
	DESC_STRING,
	'L', 0, 'P', 0, 'C', 0, 'U', 0, 'S', 0, 'B', 0,

	0x14,
	DESC_STRING,
	'U', 0, 'S', 0, 'B', 0, 'S', 0, 'e', 0, 'r', 0, 'i', 0, 'a', 0, 'l', 0,

	0x12,
	DESC_STRING,
	'D', 0, 'E', 0, 'A', 0, 'D', 0, 'C', 0, '0', 0, 'D', 0, 'E', 0,

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
static BOOL HandleClassRequest(TSetupPacket *pSetup, int *piLen, U8 **ppbData)
{
	switch (pSetup->bRequest) {

	// set line coding
	case SET_LINE_CODING:
DBG("SET_LINE_CODING\n");
		memcpy((U8 *)&LineCoding, *ppbData, 7);
		*piLen = 7;
DBG("dwDTERate=%u, bCharFormat=%u, bParityType=%u, bDataBits=%u\n",
	LineCoding.dwDTERate,
	LineCoding.bCharFormat,
	LineCoding.bParityType,
	LineCoding.bDataBits);
		break;

	// get line coding
	case GET_LINE_CODING:
DBG("GET_LINE_CODING\n");
		*ppbData = (U8 *)&LineCoding;
		*piLen = 7;
		break;

	// set control line state
	case SET_CONTROL_LINE_STATE:
		// bit0 = DTR, bit = RTS
DBG("SET_CONTROL_LINE_STATE %X\n", pSetup->wValue);
		break;

	default:
		return FALSE;
	}
	isConnected = 1;

	return TRUE;
}


/**
	Initialises the VCOM port.
	Call this function before using VCOM_putchar or VCOM_getchar
 */
void VCOM_init(void)
{
	fifo_init(&txfifo, txdata);
	fifo_init(&rxfifo, rxdata);
	fBulkInBusy = FALSE;
	fChainDone = TRUE;
}


/**
	Writes one character to VCOM port
	
	@param [in] c character to write
	@returns character written, or EOF if character could not be written
 */

int VCOM_putchar(int c)
{
	return fifo_put(&txfifo, c) ? c : EOF;
}



/**
	Reads one character from VCOM port
	
	@returns character read, or EOF if character could not be read
 */

int VCOM_getchar(void)
{
	U8 c;
	
	return fifo_get(&rxfifo, &c) ? c : EOF;
}




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
	DBG	("---------------------------------\n");
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
	VCOM_init();

	

	
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
		c = VCOM_getchar();
		if (c != EOF) {
			// show on console
			if ((c == 9) || (c == 10) || (c == 13) || ((c >= 32) && (c <= 126))) {
				DBG("%c", c);
			} else {
				DBG(".");
			}
			//VCOM_putchar(c);
		}
			
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

