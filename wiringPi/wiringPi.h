/*
 * wiringPi.h:
 *	Arduino like Wiring library for the Raspberry Pi.
 *	Copyright (c) 2012-2017 Gordon Henderson
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

#ifndef	__WIRING_PI_H__
#define	__WIRING_PI_H__

// C doesn't have true/false by default and I can never remember which
//	way round they are, so ...
//	(and yes, I know about stdbool.h but I like capitals for these and I'm old)

#ifndef	TRUE
#  define	TRUE	(1==1)
#  define	FALSE	(!TRUE)
#endif

// GCC warning suppressor

#define	UNU	__attribute__((unused))

// Mask for the bottom 64 pins which belong to the Raspberry Pi
//	The others are available for the other devices

#define	PI_GPIO_MASK	(0xFFFFFFC0)

// Handy defines

// wiringPi modes

#define	WPI_MODE_PINS		 0
#define	WPI_MODE_GPIO		 1
#define	WPI_MODE_GPIO_SYS	 2
#define	WPI_MODE_PHYS		 3
#define	WPI_MODE_PIFACE		 4
#define	WPI_MODE_UNINITIALISED	-1

// Pin modes

#define	INPUT			          0
#define	OUTPUT			        1
#define	PWM_OUTPUT		      2
#define	GPIO_CLOCK		      3
#define	SOFT_PWM_OUTPUT	    4
#define	SOFT_TONE_OUTPUT	  5
#define	PWM_TONE_OUTPUT		  6
#define	OPENDRAIN 		      7
#define	IN_INVERT           8
#define	OUT_INVERT	        9

#define	PINMUX_PWM0		      100
#define	PINMUX_PWM1		      101
#define	PINMUX_PWM2		      102
#define	PINMUX_PWM3		      103

#define	PINMUX_I2CX_FIRST   200
#define	PINMUX_I2C0_CLK     200
#define	PINMUX_I2C0_DAT     201
#define	PINMUX_I2C1_CLK     202
#define	PINMUX_I2C1_DAT     203
#define	PINMUX_I2C2_CLK     204
#define	PINMUX_I2C2_DAT     205
#define	PINMUX_I2C3_CLK     206
#define	PINMUX_I2C3_DAT     207
#define	PINMUX_I2CX_LAST    207

#define	PINMUX_SPIXM_FIRST  300
#define	PINMUX_SPI0M_INT    300
#define	PINMUX_SPI0M_CLK    301
#define	PINMUX_SPI0M_EN     302
#define	PINMUX_SPI0M_DO     303
#define	PINMUX_SPI0M_DI     304
#define	PINMUX_SPI1M_INT    305
#define	PINMUX_SPI1M_CLK    306
#define	PINMUX_SPI1M_EN     307
#define	PINMUX_SPI1M_DO     307
#define	PINMUX_SPI1M_DI     309
#define	PINMUX_SPI2M_INT    310
#define	PINMUX_SPI2M_CLK    311
#define	PINMUX_SPI2M_EN     312
#define	PINMUX_SPI2M_DO     313
#define	PINMUX_SPI2M_DI     314
#define	PINMUX_SPI3M_INT    315
#define	PINMUX_SPI3M_CLK    316
#define	PINMUX_SPI3M_EN     317
#define	PINMUX_SPI3M_DO     318
#define	PINMUX_SPI3M_DI     319
#define	PINMUX_SPIXM_LAST   319

#define PINMUX_UARTX_FIRST  400
#define PINMUX_UART1_TX     400
#define PINMUX_UART1_RX     401
#define PINMUX_UART1_CTS    402
#define PINMUX_UART1_RTS    403
#define PINMUX_UART2_TX     404 
#define PINMUX_UART2_RX     405
#define PINMUX_UART2_CTS    406
#define PINMUX_UART2_RTS    407
#define PINMUX_UART3_TX     408
#define PINMUX_UART3_RX     409
#define PINMUX_UART3_CTS    410 
#define PINMUX_UART3_RTS    411
#define PINMUX_UART4_TX     412
#define PINMUX_UART4_RX     413
#define PINMUX_UART4_CTS    414
#define PINMUX_UART4_RTS    415
#define PINMUX_UARTX_LAST   415

#define GROUP_PWM   1
#define GROUP_I2C   2
#define GROUP_SPI   3
#define GROUP_UART  4

#define	LOW			    0
#define	HIGH			  1

// Pull up/down/none

#define	PUD_OFF			 0
#define	PUD_DOWN		 1
#define	PUD_UP			 2

// PWM

#define	PWM_MODE_MS		0
#define	PWM_MODE_BAL		1

// Interrupt levels

#define	INT_EDGE_SETUP		0
#define	INT_EDGE_FALLING	1
#define	INT_EDGE_RISING		2
#define	INT_EDGE_BOTH		3

// Pi model types and version numbers
//	Intended for the GPIO program Use at your own risk.

#define	SP_MODEL_F2S		0
#define	SP_MODEL_F2P		1

#define	PI_VERSION_1		0
#define	PI_VERSION_1_1		1
#define	PI_VERSION_1_2		2
#define	PI_VERSION_2		3
#define	SP_VERSION_1		0

#define PI_LAYOUT_1     1
#define PI_LAYOUT_2     2
#define SP_LAYOUT_F2S   31
#define SP_LAYOUT_F2P   32

#define	PI_MAKER_SUNPLUS	0
#define	PI_MAKER_UNKNOWN	1

#pragma GCC diagnostic ignored "-Wstrict-aliasing" 

#define BYTE0(arg) *((unsigned char *)&(arg)+0)
#define BYTE1(arg) *((unsigned char *)&(arg)+1)
#define BYTE2(arg) *((unsigned char *)&(arg)+2)
#define BYTE3(arg) *((unsigned char *)&(arg)+3)

#define WORD0(arg) *((unsigned short *)&(arg)+0)
#define WORD1(arg) *((unsigned short *)&(arg)+1)

extern const char *piModelNames    [23] ;
extern const char *piRevisionNames [16] ;
extern const char *piMakerNames    [16] ;
extern const int   piMemorySize    [ 8] ;


//	Intended for the GPIO program Use at your own risk.

// Threads

#define	PI_THREAD(X)	void *X (UNU void *dummy)

// Failure modes

#define	WPI_FATAL	(1==1)
#define	WPI_ALMOST	(1==2)

// wiringPiNodeStruct:
//	This describes additional device nodes in the extended wiringPi
//	2.0 scheme of things.
//	It's a simple linked list for now, but will hopefully migrate to
//	a binary tree for efficiency reasons - but then again, the chances
//	of more than 1 or 2 devices being added are fairly slim, so who
//	knows....

struct wiringPiNodeStruct
{
  int     pinBase ;
  int     pinMax ;
  int     bus;

  int          fd ;	// Node specific
  unsigned int data0 ;	//  ditto
  unsigned int data1 ;	//  ditto
  unsigned int data2 ;	//  ditto
  unsigned int data3 ;	//  ditto

           void   (*pinMode)          (struct wiringPiNodeStruct *node, int pin, int mode) ;
           void   (*pullUpDnControl)  (struct wiringPiNodeStruct *node, int pin, int mode) ;
           int    (*digitalRead)      (struct wiringPiNodeStruct *node, int pin) ;
//unsigned int    (*digitalRead8)     (struct wiringPiNodeStruct *node, int pin) ;
           void   (*digitalWrite)     (struct wiringPiNodeStruct *node, int pin, int value) ;
//         void   (*digitalWrite8)    (struct wiringPiNodeStruct *node, int pin, int value) ;
           void   (*pwmWrite)         (struct wiringPiNodeStruct *node, int pin, int value) ;
           int    (*analogRead)       (struct wiringPiNodeStruct *node, int pin) ;
           void   (*analogWrite)      (struct wiringPiNodeStruct *node, int pin, int value) ;

  struct wiringPiNodeStruct *next ;
} ;

extern struct wiringPiNodeStruct *wiringPiNodes ;

// Function prototypes
//	c++ wrappers thanks to a comment by Nick Lott
//	(and others on the Raspberry Pi forums)

#ifdef __cplusplus
extern "C" {
#endif

// Data

// Internal

extern int wiringPiFailure (int fatal, const char *message, ...) ;

// Core wiringPi functions

extern struct wiringPiNodeStruct *wiringPiFindNode (int pin) ;
extern struct wiringPiNodeStruct *wiringPiNewNode  (int pinBase, int numPins) ;

extern void wiringPiVersion	(int *major, int *minor) ;
extern int  wiringPiSetup       (void) ;
extern int  wiringPiSetupSys    (void) ;
extern int  wiringPiSetupGpio   (void) ;
extern int  wiringPiSetupPhys   (void) ;

extern          void pinModeAlt          (int pin, int mode) ;
extern          void pinMode             (int pin, int mode) ;
extern          void pullUpDnControl     (int pin, int pud) ;
extern          int  digitalRead         (int pin) ;
extern          void digitalWrite        (int pin, int value) ;
extern unsigned int  digitalRead8        (int pin) ;
extern          void digitalWrite8       (int pin, int value) ;
extern          void pwmWrite            (int pin, int value) ;
extern          int  analogRead          (int pin) ;
extern          void analogWrite         (int pin, int value) ;

// PiFace specifics
//	(Deprecated)

extern int  wiringPiSetupPiFace (void) ;
extern int  wiringPiSetupPiFaceForGpioProg (void) ;	// Don't use this - for gpio program only

// On-Board Raspberry Pi hardware specific stuff

extern          int  piGpioLayout        (void) ;
extern          int  piBoardRev          (void) ;	// Deprecated
extern          void piBoardId           (int *model, int *rev, int *mem, int *maker, int *overVolted) ;
extern          int  wpiPinToGpio        (int wpiPin) ;
extern          int  physPinToGpio       (int physPin) ;
extern          int  spPinToGpio         (int physPin) ;
extern          void setPadDrive         (int group, int value) ;
extern          int  getAlt              (int pin) ;
extern          void pwmToneWrite        (int pin, int freq) ;
extern          void pwmSetMode          (int mode) ;
extern          void pwmSetRange         (unsigned int range) ;
extern          void pwmSetClock         (int port, int divisor) ;
extern          void gpioClockSet        (int pin, int freq) ;
extern unsigned int  digitalReadByte     (void) ;
extern unsigned int  digitalReadByte2    (void) ;
extern          void digitalWriteByte    (int value) ;
extern          void digitalWriteByte2   (int value) ;

// Interrupts
//	(Also Pi hardware specific)

extern int  waitForInterrupt    (int pin, int mS) ;
extern int  wiringPiISR         (int pin, int mode, void (*function)(void)) ;

// Threads

extern int  piThreadCreate      (void *(*fn)(void *)) ;
extern void piLock              (int key) ;
extern void piUnlock            (int key) ;

// Schedulling priority

extern int piHiPri (const int pri) ;

// Extras from arduino land

extern void         delay             (unsigned int howLong) ;
extern void         delayMicroseconds (unsigned int howLong) ;
extern unsigned int millis            (void) ;
extern unsigned int micros            (void) ;

#ifdef __cplusplus
}
#endif

#endif
