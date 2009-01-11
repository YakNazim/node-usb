/* isoc_ep.h */

#ifndef _ISOC_EP_H
#define _ISOC_EP_H

#define DEBUG_LED_ON(x)     IOCLR1 = (1 << x);
#define DEBUG_LED_OFF(x)    IOSET1 = (1 << x);


/*
 *  initGPIO
 *  Use pins P1.24:P1.18
 *
 *  LED1     P1.18 (OUT)
 *  LED2     P1.19 (OUT)
 *  LED3     P1.20 (OUT)
 *  LED4     P1.21 (OUT)
 *  LED5     P1.22 (OUT)
 *  LED6     P1.23 (OUT)
 *  LED7     P1.24 (OUT)
 *  LED8     P1.24 (OUT)
 */
void initGPIO(void);

#endif


