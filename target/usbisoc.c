#include "usbisoc.h"

/**
    FIXME document this
        
    @param [in|out] <paramName>    <description>

    @return 
 */
void USBSetupDMADescriptor(
		volatile U32 dmaDescriptor[], 
		volatile U32 nextDdPtr[],
		const U8 isIsocFlag, 
		const U16 maxPacketSize, 
		const U16 dmaLengthIsocNumFrames,
		void *dmaBufferStartAddress,
		U32 *isocPacketSizeMemoryAddress ) 
{
	dmaDescriptor[1] = 0;
	dmaDescriptor[0] = (U32) nextDdPtr;
	dmaDescriptor[1] |= ((maxPacketSize & 0x3FF) << 5);//Set maxPacketSize
	dmaDescriptor[1] |= (dmaLengthIsocNumFrames << 16);//aka number of ISOC packets if in ISOC mode
	if( isIsocFlag ) {
		dmaDescriptor[1] |= (1<<4);//enable isoc type
	}
	if( nextDdPtr != NULL ) {
		dmaDescriptor[1] |= (1<<2); //mark next DD as valid
	}
	dmaDescriptor[2] = (U32) dmaBufferStartAddress;
	
	if( isIsocFlag && isocPacketSizeMemoryAddress != NULL ) {
		dmaDescriptor[4] = (U32) isocPacketSizeMemoryAddress;
	}
	dmaDescriptor[3] = 0; //mark DD as valid and reset all status bits
}

/**
    FIXME document this
        
    @param [in|out] <paramName>    <description>

    @return 
 */
void USBDisableDMAForEndpoint(const U8 bEndpointNumber) {
	int idx = EP2IDX(bEndpointNumber);
	USBEpDMADis = (1<<idx);
}

/**
    FIXME document this
        
    @param [in|out] <paramName>    <description>

    @return 
 */
void USBEnableDMAForEndpoint(const U8 bEndpointNumber) {
	int idx = EP2IDX(bEndpointNumber);
	USBEpDMAEn = (1<<idx);
}

/**
    FIXME document this
        
    @param [in|out] <paramName>    <description>

    @return 
 */
void USBInitializeISOCFrameArray(U32 isocFrameArr[], const U32 numElements, const U16 startFrameNumber, const U16 defaultFrameLength) {
	U16 i;
	U16 frameNumber = startFrameNumber;
	
	for(i = 0; i < numElements; i++ ) {
		isocFrameArr[i] = (frameNumber<<16) | (1<<15) | (defaultFrameLength & 0x3FF);
		frameNumber++;
	}
}

/**
    FIXME document this
        
    @param [in|out] <paramName>    <description>

    @return 
 */
void USBSetHeadDDForDMA(const U8 bEp, volatile U32* udcaHeadArray[32], volatile const U32 *dmaDescriptorPtr) {
	udcaHeadArray[EP2IDX(bEp)] = (U32) dmaDescriptorPtr;
}

/**
    FIXME document this
        
    @param [in|out] <paramName>    <description>

    @return 
 */
void USBInitializeUSBDMA(volatile U32* udcaHeadArray[32]) {
	//set following 32 pointers to be null
	int i;
	for(i = 0; i < 32; i++ ) {
		udcaHeadArray[i] = NULL;
	}
	USBUDCAH = (U32) udcaHeadArray;
}




void USBDebugDMADescriptor(U32 dd[5]) {
	int i;
	DBG	("---------------------------------\n");
	//for(i = 0; i < 32; i++ ) {
	//	DBG("ucdaHeadArray[%d] = %X\n", i, udcaHeadArray[i]);
	//}
	U32 dd3 = dd[3];
	
	DBG("dd[0] = 0x%X\n", dd[0]);
	DBG("dd[1] = 0x%X\n", dd[1]);
	DBG("dd[2] = 0x%X\n", dd[2]);
	DBG("dd[3] = 0x%X\n", dd[3]);
	DBG("dd[4] = 0x%X\n", dd[4]);
	
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
		DBG("data underrun\n");
			break;
	case 8:
		DBG("data overrun \n");
			break;
	case 9:
		DBG("system error\n");
			break;
	}
	
	DBG("Present dma count %d\n", (dd3 >> 16));
	/*
	DBG("isocFrameArray = 0x%X\n", inputIsocFrameArray);
	DBG("isocFrameArray[0] = 0x%X\n", inputIsocFrameArray[0]);
	DBG("isocFrameArray[1] = 0x%X\n", inputIsocFrameArray[1]);
	DBG("isocFrameArray[2] = 0x%X\n", inputIsocFrameArray[2]);
	DBG("isocFrameArray[3] = 0x%X\n", inputIsocFrameArray[3]);
	DBG("isocFrameArray[4] = 0x%X\n", inputIsocFrameArray[4]);
	*/
}


/** FIXME copied from usbhw_lpc.c, don't duplicate
    Local function to wait for a device interrupt (and clear it)
        
    @param [in] dwIntr      Bitmask of interrupts to wait for   
 */
static void Wait4DevInt(U32 dwIntr)
{
    // wait for specific interrupt
    while ((USBDevIntSt & dwIntr) != dwIntr);
    // clear the interrupt bits
    USBDevIntClr = dwIntr;
}

/** FIXME copied from usbhw_lpc.c, don't duplicate
    Local function to send a command to the USB protocol engine
        
    @param [in] bCmd        Command to send
 */
static void USBHwCmd(U8 bCmd)
{
    // clear CDFULL/CCEMTY
    USBDevIntClr = CDFULL | CCEMTY;
    // write command code
    USBCmdCode = 0x00000500 | (bCmd << 16);
    Wait4DevInt(CCEMTY);
}



int USBHwISOCEPRead(U8 bEP, U8 *pbBuf, int iMaxLen)
{
    int i, idx,q;
    U32 dwData, dwLen;


    //Wait4DevInt(CDFULL);


    idx = EP2IDX(bEP);

    //USBHwCmd(CMD_EP_SELECT | idx);

    // set read enable bit for specific endpoint
    USBCtrl = RD_EN | ((bEP & 0xF) << 2);


    // wait for PKT_RDY
  /*  
    q = 0;
    do {
        dwLen = USBRxPLen;
        q++;
    } while ((dwLen & PKT_RDY) == 0 && q < 100 );
*/    


    asm volatile("nop\n");

    dwLen = USBRxPLen;
    if( (dwLen & PKT_RDY) == 0 ) {
        USBCtrl = 0;// make sure RD_EN is clear
        return(-1);
    }

    // packet valid?
    if ((dwLen & DV) == 0) {
        USBCtrl = 0;// make sure RD_EN is clear
        return -1;
    }

    // get length
    dwLen &= PKT_LNGTH_MASK;

    // get data
    dwData = 0;
    for (i = 0; i < dwLen; i++) {
        if ((i % 4) == 0) {
            dwData = USBRxData;
        }
        if ((pbBuf != NULL) && (i < iMaxLen)) {
            pbBuf[i] = dwData & 0xFF;
        }
        dwData >>= 8;
    }

    // make sure RD_EN is clear
    USBCtrl = 0;

    // select endpoint and clear buffer
    USBHwCmd(CMD_EP_SELECT | idx);
    USBHwCmd(CMD_EP_CLEAR_BUFFER);

    return dwLen;
}









