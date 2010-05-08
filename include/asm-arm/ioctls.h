#ifndef __ASM_ARM_IOCTLS_H
#define __ASM_ARM_IOCTLS_H

#include <asm/ioctl.h>

/* 0x54 is just a magic number to make these relatively unique ('T') */

#define TCGETS		0x5401
#define TCSETS		0x5402
#define TCSETSW		0x5403
#define TCSETSF		0x5404
#define TCGETA		0x5405
#define TCSETA		0x5406
#define TCSETAW		0x5407
#define TCSETAF		0x5408
#define TCSBRK		0x5409
#define TCXONC		0x540A
#define TCFLSH		0x540B
#define TIOCEXCL	0x540C
#define TIOCNXCL	0x540D
#define TIOCSCTTY	0x540E
#define TIOCGPGRP	0x540F
#define TIOCSPGRP	0x5410
#define TIOCOUTQ	0x5411
#define TIOCSTI		0x5412
#define TIOCGWINSZ	0x5413
#define TIOCSWINSZ	0x5414
#define TIOCMGET	0x5415
#define TIOCMBIS	0x5416
#define TIOCMBIC	0x5417
#define TIOCMSET	0x5418
#define TIOCGSOFTCAR	0x5419
#define TIOCSSOFTCAR	0x541A
#define FIONREAD	0x541B
#define TIOCINQ		FIONREAD
#define TIOCLINUX	0x541C
#define TIOCCONS	0x541D
#define TIOCGSERIAL	0x541E
#define TIOCSSERIAL	0x541F
#define TIOCPKT		0x5420
#define FIONBIO		0x5421
#define TIOCNOTTY	0x5422
#define TIOCSETD	0x5423
#define TIOCGETD	0x5424
#define TCSBRKP		0x5425	/* Needed for POSIX tcsendbreak() */
#define TIOCSBRK	0x5427  /* BSD compatibility */
#define TIOCCBRK	0x5428  /* BSD compatibility */
#define TIOCGSID	0x5429  /* Return the session ID of FD */
#define TIOCGPTN	_IOR('T',0x30, unsigned int) /* Get Pty Number (of pty-mux device) */
#define TIOCSPTLCK	_IOW('T',0x31, int)  /* Lock/unlock Pty */

#define FIONCLEX	0x5450  /* these numbers need to be adjusted. */
#define FIOCLEX		0x5451
#define FIOASYNC	0x5452
#define TIOCSERCONFIG	0x5453
#define TIOCSERGWILD	0x5454
#define TIOCSERSWILD	0x5455
#define TIOCGLCKTRMIOS	0x5456
#define TIOCSLCKTRMIOS	0x5457
#define TIOCSERGSTRUCT	0x5458 /* For debugging only */
#define TIOCSERGETLSR   0x5459 /* Get line status register */
#define TIOCSERGETMULTI 0x545A /* Get multiport config  */
#define TIOCSERSETMULTI 0x545B /* Set multiport config */

#define TIOCMIWAIT	0x545C	/* wait for a change on serial input line(s) */
#define TIOCGICOUNT	0x545D	/* read serial port inline interrupt counts */
#define FIOQSIZE	0x545E
/*
 * shlee 7911 define
 */
#define TIOTSMSR        0x5462      /* Read Modem Status Register */
#define TIOTGTXCNT		0x5469		/* Get Number of chars in Tx buffer*/
/* 
 *shlee 7829 for driver/char/eddy_gpio.c 
 */
 
#define INIT_PRODUCT		0x5500  /* initialize and set product */
#define GET_PRODUCTID   	0x5501  /* get product ID */
#define COMRXON		 		0x5502  /* combo model receive from serial on*/
#define COMTXON 			0x5503  /* combo model transmit to serial on */
#define INTERFACESEL  		0x5504  /* select interface and set interface */
#define RDY_LED_ON 			0x5506  /* Ready LED on */
#define RDY_LED_OFF			0x5507  /* Ready LED off */
#define RESET_READ			0x5508  /* reset switch read */
#define HW_RESET			0x5509  /* HW reset */
#define SETGPIOMODEIN		0x5510 /* set gpio mode input retrun (-1 is wrong gpio num , -2 is wrong product */
#define SETGPIOMODEOUT		0x5511 /* set gpio mode output */
#define GETGPIOMODE			0x5512 /* get gpio mode input = 1 output = 0 */
#define SETGPIOVALUEHIGH	0x5513 /* set gpio value high */ 
#define SETGPIOVALUELOW		0x5514 /* set gpio value low */
#define GETGPIOVALUE		0x5515 /* get gpio value high = 1 low = 0 */

#define GPIO_CHIP			0x5516 /* get gpio value high = 1 low = 0 */

#define SETGPIOMOD_LM		0x5517
#define GETGPIOMOD_LM		0x5518
#define SETGPIOVAL_LM		0x5519
#define GETGPIOVAL_LM		0x5520
#define SETGPIOPUL_LM		0x5521
#define GETGPIOPUL_LM		0x5522
#define SETGPIOMOD_LA		0x5523
#define GETGPIOMOD_LA		0x5524
#define SETGPIOVAL_LA		0x5525
#define GETGPIOVAL_LA		0x5526
#define SETGPIOPUL_LA		0x5527
#define GETGPIOPUL_LA		0x5528
#define SETGPIOMOD_LB		0x5529
#define GETGPIOMOD_LB		0x5530
#define SETGPIOVAL_LB		0x5531
#define GETGPIOVAL_LB		0x5532
#define SETGPIOPUL_LB		0x5533
#define GETGPIOPUL_LB		0x5534
#define SETGPIOMOD_LC		0x5535
#define GETGPIOMOD_LC		0x5536
#define SETGPIOVAL_LC		0x5537
#define GETGPIOVAL_LC		0x5538
#define SETGPIOPUL_LC		0x5539
#define GETGPIOPUL_LC		0x5540
#define SETGPIOINIT		0x5541

//<--

/*
 * shlee 8C12 define for ADC
 */
#define ADCGETVALUE			0x5600	/* get digital value */
#define ADCGETMODE			0x5601	/* get mode register setting*/
#define ADCSETMODE			0x5602	/* set mode register setting*/
#define ADCGETCHANNEL		0x5603	/* get enabled channel info*/
#define ADCSETCHANNEL		0x5604	/* enable the channel */

/* Used for packet mode */
#define TIOCPKT_DATA		 0
#define TIOCPKT_FLUSHREAD	 1
#define TIOCPKT_FLUSHWRITE	 2
#define TIOCPKT_STOP		 4
#define TIOCPKT_START		 8
#define TIOCPKT_NOSTOP		16
#define TIOCPKT_DOSTOP		32

#define TIOCSER_TEMT	0x01	/* Transmitter physically empty */

#endif
