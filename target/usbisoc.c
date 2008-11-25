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
