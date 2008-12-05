#ifndef ISOC_H_
#define ISOC_H_

#include "type.h"


#define EP2IDX(bEP) ((((bEP)&0xF)<<1)|(((bEP)&0x80)>>7))

#define USBEpDMAEn   *(volatile unsigned int *)0xE0090088
#define USBEpDMADis  *(volatile unsigned int *)0xE009008C
#define USBUDCAH     *(volatile unsigned int *)0xE0090080


#define USBDMAIntSt      *(volatile unsigned int *)0xE0090090
#define USBDMAIntEn      *(volatile unsigned int *)0xE0090094
#define USBEoTIntSt      *(volatile unsigned int *)0xE00900A0
#define USBEoTIntClr     *(volatile unsigned int *)0xE00900A4
#define USBNDDRIntSt     *(volatile unsigned int *)0xE00900AC
#define USBNDDRIntClr    *(volatile unsigned int *)0xE00900B0
#define USBSysErrIntSt   *(volatile unsigned int *)0xE00900B8
#define USBSysErrIntClr  *(volatile unsigned int *)0xE00900BC






void USBSetupDMADescriptor(
		volatile U32 dmaDescriptor[], 
		volatile U32 nextDdPtr[],
		const U8 isIsocFlag, 
		const U16 maxPacketSize, 
		const U16 dmaLengthIsocNumFrames,
		void *dmaBufferStartAddress,
		U32 *isocPacketSizeMemoryAddress );

void USBInitializeISOCFrameArray(U32 isocFrameArr[], const U32 numElements, const U16 startFrameNumber, const U16 defaultFrameLength);
void USBInitializeUSBDMA(volatile U32* udcaHeadArray[32]);
void USBSetHeadDDForDMA(const U8 bEp, volatile U32* udcaHeadArray[32], volatile const U32 *dmaDescriptorPtr);
void USBEnableDMAForEndpoint(const U8 bEndpointNumber) ;
void USBDisableDMAForEndpoint(const U8 bEndpointNumber);

#endif /*ISOC_H_*/
