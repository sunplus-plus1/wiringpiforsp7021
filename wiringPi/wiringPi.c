/*
 * wiringPi:
 *	Arduino look-a-like Wiring library for the Raspberry Pi
 *	Copyright (c) 2012-2017 Gordon Henderson
 *	Additional code for pwmSetClock by Chris Hall <chris@kchall.plus.com>
 *
 *	Thanks to code samples from Gert Jan van Loo and the
 *	BCM2835 ARM Peripherals manual, however it's missing
 *	the clock section /grr/mutter/
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as
 *    published by the Free Software Foundation, either version 3 of the
 *    License, or (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with wiringPi.
 *    If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

// Revisions:
//	19 Jul 2012:
//		Moved to the LGPL
//		Added an abstraction layer to the main routines to save a tiny
//		bit of run-time and make the clode a little cleaner (if a little
//		larger)
//		Added waitForInterrupt code
//		Added piHiPri code
//
//	 9 Jul 2012:
//		Added in support to use the /sys/class/gpio interface.
//	 2 Jul 2012:
//		Fixed a few more bugs to do with range-checking when in GPIO mode.
//	11 Jun 2012:
//		Fixed some typos.
//		Added c++ support for the .h file
//		Added a new function to allow for using my "pin" numbers, or native
//			GPIO pin numbers.
//		Removed my busy-loop delay and replaced it with a call to delayMicroseconds
//
//	02 May 2012:
//		Added in the 2 UART pins
//		Change maxPins to numPins to more accurately reflect purpose


#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <poll.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>

#include "softPwm.h"
#include "softTone.h"

#include "wiringPi.h"
#include "../version.h"

// Environment Variables

#define	ENV_DEBUG	"WIRINGPI_DEBUG"
#define	ENV_CODES	"WIRINGPI_CODES"
#define	ENV_GPIOMEM	"WIRINGPI_GPIOMEM"


// Extend wiringPi with other pin-based devices and keep track of
//	them in this structure

struct wiringPiNodeStruct *wiringPiNodes = NULL ;

#define PINMUX_CTRL_PWM01_REG           0x138
#define PINMUX_CTRL_PWM23_REG           0x13C

#define PINMUX_CTRL_SPI0M_INT_CLK_REG   0x158
#define PINMUX_CTRL_SPI0M_EN_DO_REG     0x15c
#define PINMUX_CTRL_SPI0M_DI_INT_REG    0x160
#define PINMUX_CTRL_SPI1M_CLK_EN        0x164
#define PINMUX_CTRL_SPI1M_DO_DI         0x168
#define PINMUX_CTRL_SPI2M_INT_CLK_REG   0x16c
#define PINMUX_CTRL_SPI2M_EN_DO_REG     0x170
#define PINMUX_CTRL_SPI2M_DI_INT_REG    0x174
#define PINMUX_CTRL_SPI3M_CLK_EN        0x178
#define PINMUX_CTRL_SPI3M_DO_DI         0x17c

#define PINMUX_CTRL_SPI0S_INT_CLK_REG   0x180
#define PINMUX_CTRL_SPI0S_EN_DO_REG     0x184
#define PINMUX_CTRL_SPI0S_DI_INT_REG    0x188
#define PINMUX_CTRL_SPI1S_CLK_EN        0x18C
#define PINMUX_CTRL_SPI1S_DO_DI         0x190
#define PINMUX_CTRL_SPI2S_INT_CLK_REG   0x194
#define PINMUX_CTRL_SPI2S_EN_DO_REG     0x198
#define PINMUX_CTRL_SPI2S_DI_INT_REG    0x19c
#define PINMUX_CTRL_SPI3S_CLK_EN        0x1A0
#define PINMUX_CTRL_SPI3S_DO_DI         0x1A4

#define PINMUX_CTRL_I2CM0_REG           0x1A8
#define PINMUX_CTRL_I2CM1_REG           0x1AC
#define PINMUX_CTRL_I2CM2_REG           0x1B0
#define PINMUX_CTRL_I2CM3_REG           0x1B4

#define PINMUX_CTRL_UART1_TX_RX_REG     0x1B8
#define PINMUX_CTRL_UART1_CTS_RTS_REG   0x1BC
#define PINMUX_CTRL_UART2_TX_RX_REG     0x1C0
#define PINMUX_CTRL_UART2_CTS_RTS_REG   0x1C4
#define PINMUX_CTRL_UART3_TX_RX_REG     0x1C8
#define PINMUX_CTRL_UART3_CTS_RTS_REG   0x1CC
#define PINMUX_CTRL_UART4_TX_RX_REG     0x1D0
#define PINMUX_CTRL_UART4_CTS_RTS_REG   0x1D4

#define GPIO_OUTPUT_ENABLE_REG0         0x320
#define GPIO_OUTPUT_DATA_REG0           0x340
#define GPIO_INPUT_DATA_REG0            0x360
#define GPIO_INPUT_INVERT_REG0          0x380
#define GPIO_OUTPUT_INVERT_REG0         0x3A0
#define GPIO_OENDRAIN_REG0              0x3C0

#define PWMMODE_CTRL_REG                0x7A00
#define PWMDUTY_DIV_REG                 0x7A08
#define PWM0_CONFIG_REG                 0x7A18

// Access from ARM Running Linux
//	Taken from Gert/Doms code. Some of this is not in the manual
//	that I can find )-:
//
// Updates in September 2015 - all now static variables (and apologies for the caps)
//	due to the Pi v2, v3, etc. and the new /dev/gpiomem interface

#define	PAGE_SIZE		(4*1024)
#define	BLOCK_SIZE	(0x8000)

static unsigned int usingGpioMem    = FALSE ;
static          int wiringPiSetuped = FALSE ;

// Locals to hold pointers to the hardware

static volatile unsigned int *base_addr ;
// static volatile unsigned int *timerIrqRaw ;

// Data for use with the boardId functions.
//	The order of entries here to correspond with the PI_MODEL_X
//	and PI_VERSION_X defines in wiringPi.h
//	Only intended for the gpio command - use at your own risk!

// piGpioBase:
//	The base address of the GPIO memory mapped hardware IO

#define	GPIO_PERI_BASE_7021 0x9C000000

static volatile unsigned int piGpioBase = 0 ;

enum {
ALT_IN,           // 0
ALT_OUT,
ALT_PWM0,
ALT_PWM1,
ALT_PWM2,
ALT_PWM3,
ALT_OD,
ALT_IN_INV,
ALT_OUT_INV,

ALT_I2CM0_CLK,  
ALT_I2CM0_DAT,  
ALT_I2CM1_CLK,
ALT_I2CM1_DAT,
ALT_I2CM2_CLK,
ALT_I2CM2_DAT,
ALT_I2CM3_CLK,
ALT_I2CM3_DAT,

ALT_SPI0M_INT, 
ALT_SPI0M_CLK,
ALT_SPI0M_EN,
ALT_SPI0M_DO,
ALT_SPI0M_DI,
ALT_SPI1M_INT,
ALT_SPI1M_CLK,
ALT_SPI1M_EN,
ALT_SPI1M_DO,
ALT_SPI1M_DI,
ALT_SPI2M_INT,
ALT_SPI2M_CLK,
ALT_SPI2M_EN,
ALT_SPI2M_DO,
ALT_SPI2M_DI,
ALT_SPI3M_INT,
ALT_SPI3M_CLK,
ALT_SPI3M_EN,
ALT_SPI3M_DO,
ALT_SPI3M_DI,

ALT_SPI0S_INT,  
ALT_SPI0S_CLK,
ALT_SPI0S_EN,
ALT_SPI0S_DO,
ALT_SPI0S_DI,
ALT_SPI1S_INT,
ALT_SPI1S_CLK,
ALT_SPI1S_EN,
ALT_SPI1S_DO,
ALT_SPI1S_DI,
ALT_SPI2S_INT,
ALT_SPI2S_CLK,
ALT_SPI2S_EN,
ALT_SPI2S_DO,
ALT_SPI2S_DI,
ALT_SPI3S_INT,
ALT_SPI3S_CLK,
ALT_SPI3S_EN,
ALT_SPI3S_DO,
ALT_SPI3S_DI,

ALT_UART1_TX, // 57
ALT_UART1_RX,
ALT_UART1_CTS,
ALT_UART1_RTS,
ALT_UART2_TX,
ALT_UART2_RX,
ALT_UART2_CTS,
ALT_UART2_RTS,
ALT_UART3_TX,
ALT_UART3_RX,
ALT_UART3_CTS,
ALT_UART3_RTS,
ALT_UART4_TX,
ALT_UART4_RX,
ALT_UART4_CTS,
ALT_UART4_RTS,
};

static struct {
  int reg;
  int def1;
  int def2;
} pinmuxRegs []= {
  {PINMUX_CTRL_PWM01_REG,         ALT_PWM0,      ALT_PWM1},
  {PINMUX_CTRL_PWM23_REG,         ALT_PWM2,      ALT_PWM3},
  {PINMUX_CTRL_I2CM0_REG,         ALT_I2CM0_CLK, ALT_I2CM0_DAT},
  {PINMUX_CTRL_I2CM1_REG,         ALT_I2CM1_CLK, ALT_I2CM1_DAT},
  {PINMUX_CTRL_I2CM2_REG,         ALT_I2CM2_CLK, ALT_I2CM2_DAT},
  {PINMUX_CTRL_I2CM3_REG,         ALT_I2CM3_CLK, ALT_I2CM3_DAT}, 
  {PINMUX_CTRL_SPI0M_INT_CLK_REG, ALT_SPI0M_INT, ALT_SPI0M_CLK},
  {PINMUX_CTRL_SPI0M_EN_DO_REG,   ALT_SPI0M_EN,  ALT_SPI0M_DO},  
  {PINMUX_CTRL_SPI0M_DI_INT_REG,  ALT_SPI0M_DI,  ALT_SPI1M_INT},  
  {PINMUX_CTRL_SPI1M_CLK_EN,      ALT_SPI1M_CLK, ALT_SPI1M_EN},
  {PINMUX_CTRL_SPI1M_DO_DI,       ALT_SPI1M_DO,  ALT_SPI1M_DI},  
  {PINMUX_CTRL_SPI2M_INT_CLK_REG, ALT_SPI2M_INT, ALT_SPI2M_CLK},
  {PINMUX_CTRL_SPI2M_EN_DO_REG,   ALT_SPI2M_EN,  ALT_SPI2M_DO},  
  {PINMUX_CTRL_SPI2M_DI_INT_REG,  ALT_SPI2M_DI,  ALT_SPI3M_INT},  
  {PINMUX_CTRL_SPI3M_CLK_EN,      ALT_SPI3M_CLK, ALT_SPI3M_EN},
  {PINMUX_CTRL_SPI3M_DO_DI,       ALT_SPI3M_DO,  ALT_SPI3M_DI},  
  {PINMUX_CTRL_SPI0S_INT_CLK_REG, ALT_SPI0S_INT, ALT_SPI0S_CLK},
  {PINMUX_CTRL_SPI0S_EN_DO_REG,   ALT_SPI0S_EN,  ALT_SPI0S_DO},  
  {PINMUX_CTRL_SPI0S_DI_INT_REG,  ALT_SPI0S_DI,  ALT_SPI1S_INT},  
  {PINMUX_CTRL_SPI1S_CLK_EN,      ALT_SPI1S_CLK, ALT_SPI1S_EN},
  {PINMUX_CTRL_SPI1S_DO_DI,       ALT_SPI1S_DO,  ALT_SPI1S_DI},  
  {PINMUX_CTRL_SPI2S_INT_CLK_REG, ALT_SPI2S_INT, ALT_SPI2S_CLK},
  {PINMUX_CTRL_SPI2S_EN_DO_REG,   ALT_SPI2S_EN,  ALT_SPI2S_DO},  
  {PINMUX_CTRL_SPI2S_DI_INT_REG,  ALT_SPI2S_DI,  ALT_SPI3S_INT},  
  {PINMUX_CTRL_SPI3S_CLK_EN,      ALT_SPI3S_CLK, ALT_SPI3S_EN},
  {PINMUX_CTRL_SPI3S_DO_DI,       ALT_SPI3S_DO,  ALT_SPI3S_DI},  
  {PINMUX_CTRL_UART1_TX_RX_REG,   ALT_UART1_TX,  ALT_UART1_RX},  
  {PINMUX_CTRL_UART1_CTS_RTS_REG, ALT_UART1_CTS, ALT_UART1_RTS},  
  {PINMUX_CTRL_UART2_TX_RX_REG,   ALT_UART2_TX,  ALT_UART2_RX},  
  {PINMUX_CTRL_UART2_CTS_RTS_REG, ALT_UART2_CTS, ALT_UART2_RTS},  
  {PINMUX_CTRL_UART3_TX_RX_REG,   ALT_UART3_TX,  ALT_UART3_RX},  
  {PINMUX_CTRL_UART3_CTS_RTS_REG, ALT_UART3_CTS, ALT_UART3_RTS},  
  {PINMUX_CTRL_UART4_TX_RX_REG,   ALT_UART4_TX,  ALT_UART4_RX},  
  {PINMUX_CTRL_UART4_CTS_RTS_REG, ALT_UART4_CTS, ALT_UART4_RTS},  
}; 

const char *piModelNames [] =
{
  "BPi-F2S",	// 21
  "BPi-F2P",	// 22
} ;

const char *piRevisionNames [16] =
{
  "00",
  "01",
  "02",
  "03",
  "04",
  "05",
  "06",
  "07",
  "08",
  "09",
  "10",
  "11",
  "12",
  "13",
  "14",
  "15",
} ;

const char *piMakerNames [16] =
{
  "Sunplus",	//	 3
  "Unknown",	//	 5
} ;

const int piMemorySize [8] =
{
   256,		//	 0
   512,		//	 1
  1024,		//	 2
  2048,		//	 3
  4096,		//	 4
  8192,		//	 5
     0,		//	 6
     0,		//	 7
} ;

// Time for easy calculations

static uint64_t epochMilli, epochMicro ;

// Misc

static int wiringPiMode = WPI_MODE_UNINITIALISED ;
static volatile int    pinPass = -1 ;
static pthread_mutex_t pinMutex ;

// Debugging & Return codes

int wiringPiDebug       = FALSE ;
int wiringPiReturnCodes = FALSE ;

// Use /dev/gpiomem ?

int wiringPiTryGpioMem  = FALSE ;

// sysFds:
//	Map a file descriptor from the /sys/class/gpio/gpioX/value

static int sysFds [64] =
{
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
} ;

// ISR Data

static void (*isrFunctions [64])(void) ;
static int edgeMode [64] ;


// Doing it the Arduino way with lookup tables...
//	Yes, it's probably more innefficient than all the bit-twidling, but it
//	does tend to make it all a bit clearer. At least to me!

// pinToGpio:
//	Take a Wiring pin (0 through X) and re-map it to the BCM_GPIO pin
//	Cope for 3 different board revisions here.

static int *pinToGpio ;

static int pinToGpioSP[64] =
{
    18, 17, 19, 21, 20, 22, 24, 15, // From the Original Wiki - GPIO 0 through 7:	wpi  0 -  7
    12, 13,                         // I2C  - SDA0, SCL0				wpi  8 -  9
    26, 28,                         // SPI  - CE1, CE0				wpi 10 - 11
    23, 25, 27,                     // SPI  - MOSI, MISO, SCLK			wpi 12 - 14
    14, 16,                         // UART - Tx, Rx				wpi 15 - 16
    -1, -1, -1, -1,                 // Rev 2: New GPIOs 8 though 11			wpi 17 - 20
    31, 33, 34, 36, 38,             // B+						wpi 21, 22, 23, 24, 25
    32, 35, 37, 39, 30, 29,         // B+						wpi 26, 27, 28, 29, 30, 31

    // Padding:
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // ... 47
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // ... 63
};

// physToGpio:
//	Take a physical pin (1 through 26) and re-map it to the BCM_GPIO pin
//	Cope for 2 different board revisions here.
//	Also add in the P5 connector, so the P5 pins are 3,4,5,6, so 53,54,55,56

static int *physToGpio ;

static int physToGpioSP[64] =
{
    -1,     // 0
    -1, -1, // 1, 2
    12, -1, // 3, 4
    13, -1, // 5, 6
    15, 14, // 7, 8
    -1, 16, // 9, 10
    18, 17, // 11, 12
    19, -1, // 13, 14
    21, 20, // 15, 16
    -1, 22, // 17, 18
    23, -1, // 19, 20
    25, 24, // 21, 22
    27, 26,  // 23, 24
    -1, 28,  // 25, 26
    30, 29, // 27, 28
    31, -1, // 29, 30
    33, 32, // 31, 32
    34, -1, // 33, 34
    36, 35, // 35, 36
    38, 37, // 37, 38
    -1, 39, // 39, 40
    -1, -1, -1, -1, -1, -1, -1, // ... 47
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // ... 63
};

static int SPToGpio[64] =
{
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,  8,  9, 15,  7, // 0  ~ 15
    16,  1,  0,  2,  4,  3,  5, 12,  6, 13, 10, 14, 11, 31, 30, 21, // 16 ~ 31
    26, 22, 23, 27, 24, 28, 25, 29, -1, -1, -1, -1, -1, -1, -1, -1, // 32 ~ 47   
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 48 ~ 63
};

static int PINMUX [64];

/*
 * Functions
 *********************************************************************************
 */


/*
 * wiringPiFailure:
 *	Fail. Or not.
 *********************************************************************************
 */

int wiringPiFailure (int fatal, const char *message, ...)
{
  va_list argp ;
  char buffer [1024] ;

  if (!fatal && wiringPiReturnCodes)
    return -1 ;

  va_start (argp, message) ;
    vsnprintf (buffer, 1023, message, argp) ;
  va_end (argp) ;

  fprintf (stderr, "%s", buffer) ;
  exit (EXIT_FAILURE) ;

  return 0 ;
}


/*
 * readRegister:
 *	read register
 *********************************************************************************
 */

uint64_t readRegister (volatile void *regAddr, int offset, int width)
{
    uint64_t read_result = 0;
    volatile uint8_t *virt_addr = (volatile uint8_t *)(regAddr+offset);
		switch (width) {
		case 8:
			read_result = *(volatile uint8_t*)virt_addr;
			break;
		case 16:
			read_result = *(volatile uint16_t*)virt_addr;
			break;
		case 32:
			read_result = *(volatile uint32_t*)virt_addr;
			break;
		case 64:
			read_result = *(volatile uint64_t*)virt_addr;
			break;
		}
    return read_result;
}

/*
 * readRegister:
 *	read register
 *********************************************************************************
 */

void writeRegister (volatile void *regAddr, int offset, int width, uint64_t val)
{
    volatile uint8_t *virt_addr = (volatile uint8_t *)(regAddr+offset);
		switch (width) {
		case 8:
			*(volatile uint8_t*)virt_addr = (uint8_t)val;
			break;
		case 16:
			*(volatile uint16_t*)virt_addr = (uint16_t)val;
			break;
		case 32:
			*(volatile uint32_t*)virt_addr = (uint32_t)val;
			break;
		case 64:
			*(volatile uint64_t*)virt_addr = (uint64_t)val;
			break;
		}
}

/*
 * spPinToPinmux:
 * Pinmux number to SP pin
 *********************************************************************************
 */
static int spPinToPinmux(int pin)
{
  int pinmux;
  pinmux = ((pin/8)-1)*8 + (pin%8) + 1;
  return pinmux;
}

/*
 * pinmuxToSPpin:
 * Pinmux number to SP pin
 *********************************************************************************
 */
int pinmuxToSPpin(int pin)
{
  int sppin;
  sppin = ((pin/8)+1)*8 + (pin%8) - 1;
  return sppin;
}

/*
 * spPinToGpio:
 *	Translate a SP Pin number to native GPIO pin number.
 *	Provided for external support.
 *********************************************************************************
 */

int spPinToGpio (int pin)
{
  return SPToGpio [pin & 63] ;
}

/*
 * setupCheck
 *	Another sanity check because some users forget to call the setup
 *	function. Mosty because they need feeding C drip by drip )-:
 *********************************************************************************
 */

static void setupCheck (const char *fName)
{
  if (!wiringPiSetuped)
  {
    fprintf (stderr, "%s: You have not called one of the wiringPiSetup\n"
	"  functions, so I'm aborting your program before it crashes anyway.\n", fName) ;
    exit (EXIT_FAILURE) ;
  }
}

/*
 * gpioMemCheck:
 *	See if we're using the /dev/gpiomem interface, if-so then some operations
 *	can't be done and will crash the Pi.
 *********************************************************************************
 */

static void usingGpioMemCheck (const char *what)
{
  if (usingGpioMem)
  {
    fprintf (stderr, "%s: Unable to do this when using /dev/gpiomem. Try sudo?\n", what) ;
    exit (EXIT_FAILURE) ;
  }
}

/*
 * piGpioLayout:
 *	Return a number representing the hardware revision of the board.
 *	This is not strictly the board revision but is used to check the
 *	layout of the GPIO connector - and there are 2 types that we are
 *	really interested in here. The very earliest Pi's and the
 *	ones that came after that which switched some pins ....
 *
 *	Revision 1 really means the early Model A and B's.
 *	Revision 2 is everything else - it covers the B, B+ and CM.
 *		... and the Pi 2 - which is a B+ ++  ...
 *		... and the Pi 0 - which is an A+ ...
 *
 *	The main difference between the revision 1 and 2 system that I use here
 *	is the mapping of the GPIO pins. From revision 2, the Pi Foundation changed
 *	3 GPIO pins on the (original) 26-way header - BCM_GPIO 22 was dropped and
 *	replaced with 27, and 0 + 1 - I2C bus 0 was changed to 2 + 3; I2C bus 1.
 *
 *	Additionally, here we set the piModel2 flag too. This is again, nothing to
 *	do with the actual model, but the major version numbers - the GPIO base
 *	hardware address changed at model 2 and above (not the Zero though)
 *
 *********************************************************************************
 */

static void piGpioLayoutOops (const char *why)
{
  fprintf (stderr, "Oops: Unable to determine board revision from /proc/cpuinfo\n") ;
  fprintf (stderr, " -> %s\n", why) ;
  fprintf (stderr, " ->  You'd best google the error to find out why.\n") ;
//fprintf (stderr, " ->  http://www.raspberrypi.org/phpBB3/viewtopic.php?p=184410#p184410\n") ;
  exit (EXIT_FAILURE) ;
}

int piGpioLayout (void)
{
  FILE *cpuFd ;
  char line [120] ;
  static int  gpioLayout = -1 ;

  if (gpioLayout != -1)	// No point checking twice
    return gpioLayout ;

  if ((cpuFd = popen("cat /proc/device-tree/model","r")) == NULL)
    piGpioLayoutOops ("Unable to open /proc/device-tree/model") ;

  fgets (line, 120, cpuFd);
  fclose(cpuFd);
  if (strstr(line, "BPI-F2S")!=NULL) gpioLayout = SP_LAYOUT_F2S;
  else
  if (strstr(line, "BPI-F2P")!=NULL) gpioLayout = SP_LAYOUT_F2P;
  return gpioLayout;
}

/*
 * piBoardRev:
 *	Deprecated, but does the same as piGpioLayout
 *********************************************************************************
 */

int piBoardRev (void)
{
  return piGpioLayout () ;
}

/*
 * piBoardId:
 *	Return the real details of the board we have.
 *
 *********************************************************************************
 */

void piBoardId (int *model, int *rev, int *mem, int *maker, int *warranty)
{
  int Type=0, layout ;

//	Will deal with the properly later on - for now, lets just get it going...
//  unsigned int modelNum ;

  layout = piGpioLayout () ;	// Call this first to make sure all's OK. Don't care about the result.

  if (layout==SP_LAYOUT_F2S)
    Type = SP_MODEL_F2S;
  else
  if (layout==SP_LAYOUT_F2P)
    Type = SP_MODEL_F2P;
  *model    = Type ;
  *rev      = SP_VERSION_1 ;
  *mem      = 1 ;
  *maker    = PI_MAKER_SUNPLUS;
  *warranty = 0 ;
  return;
}

/*
 * wpiPinToGpio:
 *	Translate a wiringPi Pin number to native GPIO pin number.
 *	Provided for external support.
 *********************************************************************************
 */

int wpiPinToGpio (int wpiPin)
{
  return pinToGpio [wpiPin & 63] ;
}


/*
 * physPinToGpio:
 *	Translate a physical Pin number to native GPIO pin number.
 *	Provided for external support.
 *********************************************************************************
 */

int physPinToGpio (int physPin)
{
  return physToGpio [physPin & 63] ;
}

/*
 * getGpioXAddr:
 * get offset for given pin according to pin valid bits
 *********************************************************************************
 */
void getGpioXAddr(int regBitwidth, int *pin, int *offset)
{
  *offset = (*pin/regBitwidth)*4;
  *pin = *pin%(regBitwidth);
}

/*
 * checkPinmux:
 *	Returns the ALT index. Only really of-use
 *	for the gpio readall command (I think)
 *********************************************************************************
 */

int checkPinmux(int pinmux_val)
{
  int pinmux_reg, i, pregnum;
  pregnum = sizeof(pinmuxRegs)/(3*sizeof(int));
  
  for (i=0; i<pregnum; i++)
  {
    pinmux_reg = readRegister(base_addr, pinmuxRegs[i].reg, 32);
    if (BYTE0(pinmux_reg)==pinmux_val) return pinmuxRegs[i].def1;
    if (BYTE1(pinmux_reg)==pinmux_val) return pinmuxRegs[i].def2;
  }

  return 0;
}

/*
 * getAlt:
 *	Returns the ALT bits for a given port. Only really of-use
 *	for the gpio readall command (I think)
 *********************************************************************************
 */

int getAlt (int pin)
{
  int alt, val, offset, pinmux_pin, pinmux_val;

  pin &= 63 ;
  /**/ if (wiringPiMode == WPI_MODE_PINS)
    pin = pinToGpio [pin] ;
  else if (wiringPiMode == WPI_MODE_PHYS)
    pin = physToGpio [pin] ;
  else if (wiringPiMode != WPI_MODE_GPIO)
    return 0 ;

  pinmux_val = spPinToPinmux(pin);
  pinmux_pin = checkPinmux(pinmux_val);

  if (pinmux_pin) return pinmux_pin;

  getGpioXAddr(16, &pin, &offset);

  alt = readRegister(base_addr, GPIO_OUTPUT_ENABLE_REG0+offset, 32);
  alt = alt>>pin & 0x01;

  val = readRegister(base_addr, GPIO_OENDRAIN_REG0+offset, 32);
  val = val>>pin & 0x01;

  if (val) return ALT_OD;

  if (alt==ALT_IN)
  {
    val = readRegister(base_addr, GPIO_INPUT_INVERT_REG0+offset, 32);
    val = val>>pin & 0x01;
    if (val) return ALT_IN_INV;
  }

  if (alt==ALT_OUT)
  {
    val = readRegister(base_addr, GPIO_OUTPUT_INVERT_REG0+offset, 32);
    val = val>>pin & 0x01;
    if (val) return ALT_OUT_INV;
  }

  return alt ;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter" 

/*
 * pwmSetMode:
 *	Not supported by SP7021 
 *********************************************************************************
 */

void pwmSetMode (int mode)
{
    return;
}

/*
 * setPadDrive:
 *	Set the PAD driver value
 *********************************************************************************
 */

void setPadDrive (int group, int value)
{
    return;
}

/*
 * pwmSetRange:
 *	Not supported by SP7021 
 *********************************************************************************
 */

void pwmSetRange (unsigned int range)
{
    return;
}

/*
 * pullUpDownCtrl:
 *	Control the internal pull-up/down resistors on a GPIO pin.
 *********************************************************************************
 */

void pullUpDnControl (int pin, int pud)
{
    return;
}

#pragma GCC diagnostic pop

/*
 * pwmSetClock:
 *	Set/Change the PWM clock. 
 *********************************************************************************
 */

void pwmSetClock (int pin, int freq)
{
  int pwm_sel, val, offset ;

  /**/ if (wiringPiMode == WPI_MODE_PINS)
    pin = pinToGpio [pin] ;
  else if (wiringPiMode == WPI_MODE_PHYS)
    pin = physToGpio [pin] ;
  else if (wiringPiMode != WPI_MODE_GPIO)
    return ;

  if ((wiringPiMode == WPI_MODE_PINS) || (wiringPiMode == WPI_MODE_PHYS) || (wiringPiMode == WPI_MODE_GPIO))
  {
    pwm_sel = PINMUX [pin];

    if (pwm_sel==-1) 
    {
      printf ("your select ping doesn't map to PWM");
      return;
    }

    offset = pwm_sel*4;
    val = (1/((1/202.5)*freq*256))*1000000;

    writeRegister(base_addr, PWMDUTY_DIV_REG+offset, 32, val);
  }
}

/*
 * wiringPiFindNode:
 *      Locate our device node
 *********************************************************************************
 */

struct wiringPiNodeStruct *wiringPiFindNode (int pin)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;

  while (node != NULL)
    if ((pin >= node->pinBase) && (pin <= node->pinMax))
      return node ;
    else
      node = node->next ;

  return NULL ;
}


/*
 * wiringPiNewNode:
 *	Create a new GPIO node into the wiringPi handling system
 *********************************************************************************
 */

static         void pinModeDummy             (UNU struct wiringPiNodeStruct *node, UNU int pin, UNU int mode)  { return ; }
static         void pullUpDnControlDummy     (UNU struct wiringPiNodeStruct *node, UNU int pin, UNU int pud)   { return ; }
//static unsigned int digitalRead8Dummy        (UNU struct wiringPiNodeStruct *node, UNU int UNU pin)            { return 0 ; }
//static         void digitalWrite8Dummy       (UNU struct wiringPiNodeStruct *node, UNU int pin, UNU int value) { return ; }
static          int digitalReadDummy         (UNU struct wiringPiNodeStruct *node, UNU int UNU pin)            { return LOW ; }
static         void digitalWriteDummy        (UNU struct wiringPiNodeStruct *node, UNU int pin, UNU int value) { return ; }
static         void pwmWriteDummy            (UNU struct wiringPiNodeStruct *node, UNU int pin, UNU int value) { return ; }
static          int analogReadDummy          (UNU struct wiringPiNodeStruct *node, UNU int pin)            { return 0 ; }
static         void analogWriteDummy         (UNU struct wiringPiNodeStruct *node, UNU int pin, UNU int value) { return ; }

struct wiringPiNodeStruct *wiringPiNewNode (int pinBase, int numPins)
{
  int    pin ;
  struct wiringPiNodeStruct *node ;

// Minimum pin base is 64

  if (pinBase < 64)
    (void)wiringPiFailure (WPI_FATAL, "wiringPiNewNode: pinBase of %d is < 64\n", pinBase) ;

// Check all pins in-case there is overlap:

  for (pin = pinBase ; pin < (pinBase + numPins) ; ++pin)
    if (wiringPiFindNode (pin) != NULL)
      (void)wiringPiFailure (WPI_FATAL, "wiringPiNewNode: Pin %d overlaps with existing definition\n", pin) ;

  node = (struct wiringPiNodeStruct *)calloc (sizeof (struct wiringPiNodeStruct), 1) ;	// calloc zeros
  if (node == NULL)
    (void)wiringPiFailure (WPI_FATAL, "wiringPiNewNode: Unable to allocate memory: %s\n", strerror (errno)) ;

  node->pinBase          = pinBase ;
  node->pinMax           = pinBase + numPins - 1 ;
  node->pinMode          = pinModeDummy ;
  node->pullUpDnControl  = pullUpDnControlDummy ;
  node->digitalRead      = digitalReadDummy ;
//node->digitalRead8     = digitalRead8Dummy ;
  node->digitalWrite     = digitalWriteDummy ;
//node->digitalWrite8    = digitalWrite8Dummy ;
  node->pwmWrite         = pwmWriteDummy ;
  node->analogRead       = analogReadDummy ;
  node->analogWrite      = analogWriteDummy ;
  node->next             = wiringPiNodes ;
  wiringPiNodes          = node ;

  return node ;
}


#ifdef notYetReady
/*
 * pinED01:
 * pinED10:
 *	Enables edge-detect mode on a pin - from a 0 to a 1 or 1 to 0
 *	Pin must already be in input mode with appropriate pull up/downs set.
 *********************************************************************************
 */

void pinEnableED01Pi (int pin)
{
  pin = pinToGpio [pin & 63] ;
}
#endif

/*
 * pinmux_Set:
 * Mapping pwm to gpio
 *********************************************************************************
 */
static void pinmux_Set(int mode, int pval)
{
  int offset, mask, val, pwm_ctrl;
  int group = 0;

  if (mode>=PINMUX_PWM0 && mode<=PINMUX_PWM3)
  {
    mode = mode - PINMUX_PWM0;
    group = GROUP_PWM;
  }
  else
  if (mode>=PINMUX_I2CX_FIRST && mode<=PINMUX_I2CX_LAST)
  {
    mode = mode - PINMUX_I2CX_FIRST;
    group = GROUP_I2C;
  }
  else
  if (mode>=PINMUX_SPIXM_FIRST && mode<=PINMUX_SPIXM_LAST)
  {
    mode = mode - PINMUX_SPIXM_FIRST;
    group = GROUP_SPI;
  }
  else
  if (mode>=PINMUX_UARTX_FIRST && mode<=PINMUX_UARTX_LAST)
  {
    mode = mode - PINMUX_UARTX_FIRST;
    group = GROUP_UART;
  }

  switch (mode%4)
  {
    case 0:
    case 2:
      mask = 0x3f << 16;
      break;
    case 1:
    case 3:
      pval = pval << 8;
      mask = 0x3f << 24;
      break;
    default:
      return;
  }
  offset = (mode/2)*4;
  val = mask | pval;

  if (group==GROUP_PWM) 
  {
    writeRegister(base_addr, PINMUX_CTRL_PWM01_REG+offset, 32, val);

    pwm_ctrl = readRegister(base_addr, PWMMODE_CTRL_REG, 32);

    // set pwm enable

    if ((pwm_ctrl>>mode & 0x01)==0)
    {
      pwm_ctrl = pwm_ctrl | 1<<mode;
      writeRegister(base_addr, PWMMODE_CTRL_REG, 32, pwm_ctrl);
    }
  }
  else
  if (group==GROUP_I2C) 
  {
    writeRegister(base_addr, PINMUX_CTRL_I2CM0_REG+offset, 32, val);
  }
  else
  if (group==GROUP_SPI) 
  {
    writeRegister(base_addr, PINMUX_CTRL_SPI0M_INT_CLK_REG+offset, 32, val);
  }
  else
  if (group==GROUP_UART) 
  {
    writeRegister(base_addr, PINMUX_CTRL_UART1_TX_RX_REG+offset, 32, val);
  }
}

/*
 * pinMode:
 *	Sets the mode of a pin to be input, output or PWM output
 *********************************************************************************
 */

void pinMode (int pin, int mode)
{
  int    offset, val ;
  struct wiringPiNodeStruct *node = wiringPiNodes ;
  int origPin = pin, spPin, i , pinmux_pin;

  setupCheck ("pinMode") ;

  if ((pin & PI_GPIO_MASK) == 0)		// On-board pin
  {
    /**/ if (wiringPiMode == WPI_MODE_PINS)
      pin = pinToGpio [pin] ;
    else if (wiringPiMode == WPI_MODE_PHYS)
      pin = physToGpio [pin] ;
    else if (wiringPiMode != WPI_MODE_GPIO)
      return ;

    spPin = pin;
    softPwmStop  (origPin) ;
    softToneStop (origPin) ;

    /**/ if (mode == INPUT || mode == IN_INVERT) {
      getGpioXAddr(16, &pin, &offset);
      WORD0(val) = 0<<pin;
      WORD1(val) = 1<<pin;
      
      writeRegister(base_addr, GPIO_OUTPUT_ENABLE_REG0+offset, 32, val);
      writeRegister(base_addr, GPIO_OENDRAIN_REG0+offset, 32, val);

      if (mode == IN_INVERT) 
      {
        WORD0(val) = 1<<pin;
        writeRegister(base_addr, GPIO_INPUT_INVERT_REG0+offset, 32, val);
      }
      else
      {
        WORD0(val) = 0<<pin;
        writeRegister(base_addr, GPIO_INPUT_INVERT_REG0+offset, 32, val);
      }
    }
    else if (mode == OUTPUT || mode == OPENDRAIN || mode == OUT_INVERT) {
      getGpioXAddr(16, &pin, &offset);
      WORD0(val) = 1<<pin;
      WORD1(val) = 1<<pin;

      if (mode == OPENDRAIN) 
        writeRegister(base_addr, GPIO_OENDRAIN_REG0+offset, 32, val);

      writeRegister(base_addr, GPIO_OUTPUT_ENABLE_REG0+offset, 32, val);

      if (mode == OUT_INVERT) 
        writeRegister(base_addr, GPIO_OUTPUT_INVERT_REG0+offset, 32, val);
      else
      {
        WORD0(val) = 0<<pin;
        writeRegister(base_addr, GPIO_OUTPUT_INVERT_REG0+offset, 32, val);
      }
    }
    else if (mode == SOFT_PWM_OUTPUT)
      softPwmCreate (origPin, 0, 100) ;
    else if (mode == SOFT_TONE_OUTPUT)
      softToneCreate (origPin) ;
    else if (mode == PWM_TONE_OUTPUT)
    {
      pinMode (origPin, PWM_OUTPUT) ;	// Call myself to enable PWM mode
      pwmSetMode (PWM_MODE_MS) ;
    }
    else if (mode >= PINMUX_PWM0 && mode <= PINMUX_PWM3)
    {
      usingGpioMemCheck ("pinMode PWM") ;

      pinmux_pin = PINMUX[spPin];
      if (pinmux_pin>=0) {
        printf ("SP pin %d is used by pwm%d.\n", spPin, pinmux_pin);
        return;
      }

      pinmux_Set(mode, spPinToPinmux(spPin));
      PINMUX[spPin] = mode;

      for (i=0; i<64; i++)
      {
        if (i==spPin) continue;
        if (PINMUX[spPin]==pinmux_pin) 
        {
          PINMUX[spPin]=-1;
          break;
        }
      }
    }
    else if ((mode >= PINMUX_I2CX_FIRST && mode <= PINMUX_I2CX_LAST)   ||
             (mode >= PINMUX_SPIXM_FIRST && mode <= PINMUX_SPIXM_LAST) ||
             (mode >= PINMUX_UARTX_FIRST && mode <= PINMUX_UARTX_LAST) 
             )
    {
      pinmux_Set(mode, spPinToPinmux(spPin));
    }
  }
  else
  {
    if ((node = wiringPiFindNode (pin)) != NULL)
      node->pinMode (node, pin, mode) ;
    return ;
  }
}

/*
 * digitalRead:
 *	Read the value of a given Pin, returning HIGH or LOW
 *********************************************************************************
 */

int digitalRead (int pin)
{
  char c ;
  int offset, addr, width, val, orgpin=pin;

  struct wiringPiNodeStruct *node = wiringPiNodes ;
  if ((pin & PI_GPIO_MASK) == 0)		// On-Board Pin
  {
    /**/
    if (wiringPiMode == WPI_MODE_GPIO_SYS)	// Sys mode
    {
      if (sysFds [pin] == -1) {
        printf ("pin %d doesn't exported.\n", pin);
        return LOW ;
      }

      lseek (sysFds [pin], 0L, SEEK_SET) ;
      read (sysFds [pin], &c, 1) ;
      return (c == '0') ? LOW : HIGH ;
    }
    else if (wiringPiMode == WPI_MODE_PINS)
      pin = pinToGpio [pin] ;
    else if (wiringPiMode == WPI_MODE_PHYS)
      pin = physToGpio [pin] ;
    else if (wiringPiMode != WPI_MODE_GPIO)
      return LOW ;

    val = getAlt(orgpin);
    width = 16;

    switch(val)
    {
      case ALT_IN:
      case ALT_IN_INV:
        addr = GPIO_INPUT_DATA_REG0; // input data register
        width = 32;
        break;
      case ALT_OUT_INV:
      case ALT_OUT:
      case ALT_PWM0:
      case ALT_PWM1:
      case ALT_PWM2:
      case ALT_PWM3:
      case ALT_OD:
        addr = GPIO_OUTPUT_DATA_REG0;
        break;
      default:
        return LOW;
    }

    getGpioXAddr(width, &pin, &offset); 
  
    val = readRegister(base_addr, addr+offset, 32);

    val = val>>pin & 0x01;

    if (val != 0)
      return HIGH ;
    else
      return LOW ;
  }
  else
  {
    if ((node = wiringPiFindNode (pin)) == NULL)
      return LOW ;
    return node->digitalRead (node, pin) ;
  }
}


/*
 * digitalRead8:
 *	Read 8-bits (a byte) from given start pin.
 *********************************************************************************

unsigned int digitalRead8 (int pin)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;

  if ((pin & PI_GPIO_MASK) == 0)		// On-Board Pin
    return 0 ;
  else
  {
    if ((node = wiringPiFindNode (pin)) == NULL)
      return LOW ;
    return node->digitalRead8 (node, pin) ;
  }
}
 */


/*
 * digitalWrite:
 *	Set an output bit
 *********************************************************************************
 */

void digitalWrite (int pin, int value)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;
  int offset, val;

  if ((pin & PI_GPIO_MASK) == 0)		// On-Board Pin
  {
    /**/ if (wiringPiMode == WPI_MODE_GPIO_SYS)	// Sys mode
    {
      if (sysFds [pin] != -1)
      {
        if (value == LOW) {
          if (write (sysFds [pin], "0\n", 2)<0)
            perror("write: ");
        }
        else {
          if (write (sysFds [pin], "1\n", 2)<0)
            perror("write: ");
        }
      }
      else
      {
        printf ("pin %d doesn't exported.\n", pin);
      }
      return ;
    }
    else if (wiringPiMode == WPI_MODE_PINS)
      pin = pinToGpio [pin] ;
    else if (wiringPiMode == WPI_MODE_PHYS)
      pin = physToGpio [pin] ;
    else if (wiringPiMode != WPI_MODE_GPIO)
      return ;

    getGpioXAddr(16, &pin, &offset);

    WORD0(val) = value<<pin;
    WORD1(val) = 1<<pin;

    writeRegister(base_addr, GPIO_OUTPUT_DATA_REG0+offset, 32, val);
  }
  else
  {
    printf ("wiringPiFindNode\n");
    if ((node = wiringPiFindNode (pin)) != NULL)
      node->digitalWrite (node, pin, value) ;
  }
}


/*
 * digitalWrite8:
 *	Set an output 8-bit byte on the device from the given pin number
 *********************************************************************************

void digitalWrite8 (int pin, int value)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;

  if ((pin & PI_GPIO_MASK) == 0)		// On-Board Pin
    return ;
  else
  {
    if ((node = wiringPiFindNode (pin)) != NULL)
      node->digitalWrite8 (node, pin, value) ;
  }
}
 */


/*
 * pwmWrite:
 *	Set an output PWM value
 *********************************************************************************
 */

void pwmWrite (int pin, int value)
{
  int offset;
  struct wiringPiNodeStruct *node = wiringPiNodes ;

  setupCheck ("pwmWrite") ;

  if ((pin & PI_GPIO_MASK) == 0)		// On-Board Pin
  {
    /**/ if (wiringPiMode == WPI_MODE_PINS)
      pin = pinToGpio [pin] ;
    else if (wiringPiMode == WPI_MODE_PHYS)
      pin = physToGpio [pin] ;
    else if (wiringPiMode != WPI_MODE_GPIO)
      return ;

    usingGpioMemCheck ("pwmWrite") ;

    offset = PINMUX [pin]*4;
    BYTE0(value) = value;
    BYTE1(value) = PINMUX [pin];

    writeRegister(base_addr, PWM0_CONFIG_REG+offset, 32, value);
  }
  else
  {
    if ((node = wiringPiFindNode (pin)) != NULL)
      node->pwmWrite (node, pin, value) ;
  }
}


/*
 * analogRead:
 *	Read the analog value of a given Pin.
 *	There is no on-board Pi analog hardware,
 *	so this needs to go to a new node.
 *********************************************************************************
 */

int analogRead (int pin)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;

  if ((node = wiringPiFindNode (pin)) == NULL)
    return 0 ;
  else
    return node->analogRead (node, pin) ;
}


/*
 * analogWrite:
 *	Write the analog value to the given Pin.
 *	There is no on-board Pi analog hardware,
 *	so this needs to go to a new node.
 *********************************************************************************
 */

void analogWrite (int pin, int value)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;

  if ((node = wiringPiFindNode (pin)) == NULL)
    return ;

  node->analogWrite (node, pin, value) ;
}


/*
 * pwmToneWrite:
 *	Pi Specific.
 *      Output the given frequency on the Pi's PWM pin
 *********************************************************************************
 */

void pwmToneWrite (int pin, int freq)
{
  setupCheck ("pwmToneWrite") ;

  if (freq == 0)
    pwmWrite (pin, 0) ;             // Off
  else
  {
    pwmSetClock(pin, freq);
    pwmWrite(pin, 128);
  }
}

/*
 * digitalWriteByte:
 * digitalReadByte:
 *	Pi Specific
 *	Write an 8-bit byte to the first 8 GPIO pins - try to do it as
 *	fast as possible.
 *	However it still needs 2 operations to set the bits, so any external
 *	hardware must not rely on seeing a change as there will be a change
 *	to set the outputs bits to zero, then another change to set the 1's
 *	Reading is just bit fiddling.
 *	These are wiringPi pin numbers 0..7
 *********************************************************************************
 */

void digitalWriteByte (const int value)
{
  int mask = 1 ;
  int pin ;

  for (pin = 0 ; pin < 8 ; ++pin)
  {
    digitalWrite (pinToGpio [pin], value & mask) ;
    mask <<= 1 ;
  }
}

unsigned int digitalReadByte (void)
{
  int pin, x ;
  uint32_t data = 0 ;

  for (pin = 0 ; pin < 8 ; ++pin)
  {
    x = digitalRead (pinToGpio [pin]) ;
    data = (data << 1) | x ;
  }
  return data ;
}


/*
 * digitalWriteByte2:
 * digitalReadByte2:
 *	Pi Specific
 *	Write an 8-bit byte to the second set of 8 GPIO pins. This is marginally
 *	faster than the first lot as these are consecutive BCM_GPIO pin numbers.
 *	However they overlap with the original read/write bytes.
 *********************************************************************************
 */

void digitalWriteByte2 (const int value)
{
  register int mask = 1 ;
  register int pin ;

  for (pin = 20 ; pin < 28 ; ++pin)
  {
    digitalWrite (pin, value & mask) ;
    mask <<= 1 ;
  }
}

unsigned int digitalReadByte2 (void)
{
  int pin, x ;
  uint32_t data = 0 ;

  for (pin = 20 ; pin < 28 ; ++pin)
  {
    x = digitalRead (pin) ;
    data = (data << 1) | x ;
  }

  return data ;
}

/*
 * initialiseEpoch:
 *	Initialise our start-of-time variable to be the current unix
 *	time in milliseconds and microseconds.
 *********************************************************************************
 */

static void initialiseEpoch (void)
{
#ifdef	OLD_WAY
  struct timeval tv ;

  gettimeofday (&tv, NULL) ;
  epochMilli = (uint64_t)tv.tv_sec * (uint64_t)1000    + (uint64_t)(tv.tv_usec / 1000) ;
  epochMicro = (uint64_t)tv.tv_sec * (uint64_t)1000000 + (uint64_t)(tv.tv_usec) ;
#else
  struct timespec ts ;

  clock_gettime (CLOCK_MONOTONIC_RAW, &ts) ;
  epochMilli = (uint64_t)ts.tv_sec * (uint64_t)1000    + (uint64_t)(ts.tv_nsec / 1000000L) ;
  epochMicro = (uint64_t)ts.tv_sec * (uint64_t)1000000 + (uint64_t)(ts.tv_nsec /    1000L) ;
#endif
}

/*
 * waitForInterrupt:
 *	Pi Specific.
 *	Wait for Interrupt on a GPIO pin. Because SP7021 doesn't support GPIO edge
 *	This is actually done by using polling register
 *	the wiringPi access mode in-use. Maybe sometime it might get a better
 *	way for a bit more efficiency.
 *********************************************************************************
 */

int waitForInterrupt (int pin, int mS)
{
  int pre_val, val ;

  initialiseEpoch();
  pre_val = digitalRead(pin) ;
  while (1)
  {    
    val = digitalRead(pin) ;
    // printf ("%d, edage = %d, pre_val = %d, val = %d \n", pin, edgeMode[pin], pre_val, val) ;

    if (edgeMode[pin]==INT_EDGE_BOTH)
    {
      if (pre_val != val) break ;
    }
    else
    if (edgeMode[pin]==INT_EDGE_RISING)
    {
      if (pre_val==0 && val==1) break ;
    }
    else
    if (edgeMode[pin]==INT_EDGE_FALLING)
    {
      if (pre_val==1 && val==0) break ;
    }
    if (mS>0 && (int)millis()>=mS) 
    {
      return 0;
    }
    if (edgeMode[pin] != INT_EDGE_BOTH && pre_val != val) pre_val = val ;

    delay(2);
  }
  return 1 ;
}


/*
 * interruptHandler:
 *	This is a thread and gets started to wait for the interrupt we're
 *	hoping to catch. It will call the user-function when the interrupt
 *	fires.
 *********************************************************************************
 */

static void *interruptHandler (UNU void *arg)
{
  int myPin ;

  (void)piHiPri (55) ;	// Only effective if we run as root

  myPin   = pinPass ;
  pinPass = -1 ;

  while(1)
  {
    if (waitForInterrupt (myPin, -1) > 0)
      isrFunctions [myPin] () ;
  }
  return NULL;
}

/*
 * wiringPiISR:
 *	Pi Specific.
 *	Take the details and create an interrupt handler that will do a call-
 *	back to the user supplied function.
 *********************************************************************************
 */

int wiringPiISR (int pin, int mode, void (*function)(void))
{
  pthread_t threadId ;

  if ((pin < 0) || (pin > 63))
    return wiringPiFailure (WPI_FATAL, "wiringPiISR: pin must be 0-63 (%d)\n", pin) ;

// Now export the pin and set the right edge
//	We're going to use the gpio program to do this, so it assumes
//	a full installation of wiringPi. It's a bit 'clunky', but it
//	is a way that will work when we're running in "Sys" mode, as
//	a non-root user. (without sudo)

  edgeMode[pin] = mode;
  isrFunctions [pin] = function ;

  pthread_mutex_lock (&pinMutex) ;  
    pinPass = pin ;
    pthread_create (&threadId, NULL, interruptHandler, NULL) ;
    while (pinPass != -1)
      delay (1) ;
  pthread_mutex_unlock (&pinMutex) ;

  return 0 ;
}

/*
 * delay:
 *	Wait for some number of milliseconds
 *********************************************************************************
 */

void delay (unsigned int howLong)
{
  struct timespec sleeper, dummy ;

  sleeper.tv_sec  = (time_t)(howLong / 1000) ;
  sleeper.tv_nsec = (long)(howLong % 1000) * 1000000 ;

  nanosleep (&sleeper, &dummy) ;
}


/*
 * delayMicroseconds:
 *	This is somewhat intersting. It seems that on the Pi, a single call
 *	to nanosleep takes some 80 to 130 microseconds anyway, so while
 *	obeying the standards (may take longer), it's not always what we
 *	want!
 *
 *	So what I'll do now is if the delay is less than 100uS we'll do it
 *	in a hard loop, watching a built-in counter on the ARM chip. This is
 *	somewhat sub-optimal in that it uses 100% CPU, something not an issue
 *	in a microcontroller, but under a multi-tasking, multi-user OS, it's
 *	wastefull, however we've no real choice )-:
 *
 *      Plan B: It seems all might not be well with that plan, so changing it
 *      to use gettimeofday () and poll on that instead...
 *********************************************************************************
 */

void delayMicrosecondsHard (unsigned int howLong)
{
  struct timeval tNow, tLong, tEnd ;

  gettimeofday (&tNow, NULL) ;
  tLong.tv_sec  = howLong / 1000000 ;
  tLong.tv_usec = howLong % 1000000 ;
  timeradd (&tNow, &tLong, &tEnd) ;

  while (timercmp (&tNow, &tEnd, <))
    gettimeofday (&tNow, NULL) ;
}

void delayMicroseconds (unsigned int howLong)
{
  struct timespec sleeper ;
  unsigned int uSecs = howLong % 1000000 ;
  unsigned int wSecs = howLong / 1000000 ;

  /**/ if (howLong ==   0)
    return ;
  else if (howLong  < 100)
    delayMicrosecondsHard (howLong) ;
  else
  {
    sleeper.tv_sec  = wSecs ;
    sleeper.tv_nsec = (long)(uSecs * 1000L) ;
    nanosleep (&sleeper, NULL) ;
  }
}


/*
 * millis:
 *	Return a number of milliseconds as an unsigned int.
 *	Wraps at 49 days.
 *********************************************************************************
 */

unsigned int millis (void)
{
  uint64_t now ;

#ifdef	OLD_WAY
  struct timeval tv ;

  gettimeofday (&tv, NULL) ;
  now  = (uint64_t)tv.tv_sec * (uint64_t)1000 + (uint64_t)(tv.tv_usec / 1000) ;

#else
  struct  timespec ts ;

  clock_gettime (CLOCK_MONOTONIC_RAW, &ts) ;
  now  = (uint64_t)ts.tv_sec * (uint64_t)1000 + (uint64_t)(ts.tv_nsec / 1000000L) ;
#endif

  return (uint32_t)(now - epochMilli) ;
}


/*
 * micros:
 *	Return a number of microseconds as an unsigned int.
 *	Wraps after 71 minutes.
 *********************************************************************************
 */

unsigned int micros (void)
{
  uint64_t now ;
#ifdef	OLD_WAY
  struct timeval tv ;

  gettimeofday (&tv, NULL) ;
  now  = (uint64_t)tv.tv_sec * (uint64_t)1000000 + (uint64_t)tv.tv_usec ;
#else
  struct  timespec ts ;

  clock_gettime (CLOCK_MONOTONIC_RAW, &ts) ;
  now  = (uint64_t)ts.tv_sec * (uint64_t)1000000 + (uint64_t)(ts.tv_nsec / 1000) ;
#endif


  return (uint32_t)(now - epochMicro) ;
}

/*
 * wiringPiVersion:
 *	Return our current version number
 *********************************************************************************
 */

void wiringPiVersion (int *major, int *minor)
{
  *major = VERSION_MAJOR ;
  *minor = VERSION_MINOR ;
}


/*
 * wiringPiSetup:
 *	Must be called once at the start of your program execution.
 *
 * Default setup: Initialises the system into wiringPi Pin mode and uses the
 *	memory mapped hardware directly.
 *
 * Changed now to revert to "gpio" mode if we're running on a Compute Module.
 *********************************************************************************
 */

int wiringPiSetup (void)
{
  int   fd, npwm, val, spPin, Ret;
  char  path[128], cmd[512];
  int   model, rev, mem, maker, overVolted ;
  struct stat sb;

// It's actually a fatal error to call any of the wiringPiSetup routines more than once,
//	(you run out of file handles!) but I'm fed-up with the useless twats who email
//	me bleating that there is a bug in my code, so screw-em.

  if (wiringPiSetuped)
    return 0 ;

  memset(edgeMode, -1, 64*sizeof(int));
  memset(PINMUX, -1, 64*sizeof(int));

  wiringPiSetuped = TRUE ;

  if (getenv (ENV_DEBUG) != NULL)
    wiringPiDebug = TRUE ;

  if (getenv (ENV_CODES) != NULL)
    wiringPiReturnCodes = TRUE ;

  if (wiringPiDebug)
    printf ("wiringPi: wiringPiSetup called\n") ;

// Get the board ID information. We're not really using the information here,
//	but it will give us information like the GPIO layout scheme (2 variants
//	on the older 26-pin Pi's) and the GPIO peripheral base address.
//	and if we're running on a compute module, then wiringPi pin numbers
//	don't really many anything, so force native BCM mode anyway.

  piBoardId (&model, &rev, &mem, &maker, &overVolted) ;

  wiringPiMode = WPI_MODE_PINS ;

  if (piGpioLayout () == SP_LAYOUT_F2S)
  {
     pinToGpio =  pinToGpioSP ;
    physToGpio = physToGpioSP ;
  }
  else
  if (piGpioLayout () == SP_LAYOUT_F2P)
  {
     pinToGpio =  pinToGpioSP ;
    physToGpio = physToGpioSP ;
  }

// ...

  piGpioBase = GPIO_PERI_BASE_7021 ;

// Open the master /dev/ memory control device
// Device strategy: December 2016:
//	Try /dev/mem. If that fails, then
//	try /dev/gpiomem. If that fails then game over.

  if ((fd = open ("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC)) < 0)
  {
    if ((fd = open ("/dev/gpiomem", O_RDWR | O_SYNC | O_CLOEXEC) ) >= 0)	// We're using gpiomem
    {
      piGpioBase   = 0 ;
      usingGpioMem = TRUE ;
    }
    else
      return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: Unable to open /dev/mem or /dev/gpiomem: %s.\n"
	"  Aborting your program because if it can not access the GPIO\n"
	"  hardware then it most certianly won't work\n"
	"  Try running with sudo?\n", strerror (errno)) ;
  }

  base_addr = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, piGpioBase) ;
  if (base_addr == MAP_FAILED)
    return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: mmap (GPIO) failed: %s\n", strerror (errno)) ;

// enable PWM

  if (stat("/sys/class/pwm/pwmchip0", &sb) == 0) 
  {
    for (npwm=0; npwm<4; npwm++)
    {
      sprintf(path, "/sys/class/pwm/pwmchip0/pwm%d", npwm);
      if (stat(path, &sb) != 0) {
        Ret = FALSE;
        while(1) 
        {
          sprintf(cmd, "echo %d 2>/dev/null > /sys/class/pwm/pwmchip0/export", npwm);
          // printf ("cmd = %s\n", cmd);
          if (system(cmd)) break;
          sprintf(cmd, "echo 5000 2>/dev/null > %s/period",path);
          // printf ("cmd = %s\n", cmd);
          if (system(cmd)) break;
          sprintf(cmd, "echo 4000 2>/dev/null > %s/duty_cycle", path);
          // printf ("cmd = %s\n", cmd);
          if (system(cmd)) break;
          sprintf(cmd, "echo 1 2>/dev/null > %s/enable", path);
          // printf ("cmd = %s\n", cmd);
          if (system(cmd)) break;
          Ret = TRUE;
          break;
        }
        if (Ret==FALSE) 
        {
          printf ("Please using sudo to run.\n");
          exit(1);
          break;
        }
      }
    }

    val = readRegister(base_addr, PINMUX_CTRL_PWM01_REG, 32);
    
    spPin = pinmuxToSPpin(BYTE0(val));
    PINMUX[spPin] = 0;

    spPin = pinmuxToSPpin(BYTE1(val));
    PINMUX[spPin] = 1;

    val = readRegister(base_addr, PINMUX_CTRL_PWM23_REG, 32);

    spPin = pinmuxToSPpin(BYTE0(val));
    PINMUX[spPin] = 2;

    spPin = pinmuxToSPpin(BYTE1(val));
    PINMUX[spPin] = 3;
  }

  initialiseEpoch () ;

  return 0 ;
}

/*
 * wiringPiSetupGpio:
 *	Must be called once at the start of your program execution.
 *
 * GPIO setup: Initialises the system into GPIO Pin mode and uses the
 *	memory mapped hardware directly.
 *********************************************************************************
 */

int wiringPiSetupGpio (void)
{
  (void)wiringPiSetup () ;

  if (wiringPiDebug)
    printf ("wiringPi: wiringPiSetupGpio called\n") ;

  wiringPiMode = WPI_MODE_GPIO ;

  return 0 ;
}


/*
 * wiringPiSetupPhys:
 *	Must be called once at the start of your program execution.
 *
 * Phys setup: Initialises the system into Physical Pin mode and uses the
 *	memory mapped hardware directly.
 *********************************************************************************
 */

int wiringPiSetupPhys (void)
{
  (void)wiringPiSetup () ;

  if (wiringPiDebug)
    printf ("wiringPi: wiringPiSetupPhys called\n") ;

  wiringPiMode = WPI_MODE_PHYS ;

  return 0 ;
}


/*
 * wiringPiSetupSys:
 *	Must be called once at the start of your program execution.
 *
 * Initialisation (again), however this time we are using the /sys/class/gpio
 *	interface to the GPIO systems - slightly slower, but always usable as
 *	a non-root user, assuming the devices are already exported and setup correctly.
 */

int wiringPiSetupSys (void)
{
  int pin, spin ;
  int layout;
  char fName [128] ;

  if (wiringPiSetuped)
    return 0 ;

  wiringPiSetuped = TRUE ;

  if (getenv (ENV_DEBUG) != NULL)
    wiringPiDebug = TRUE ;

  if (getenv (ENV_CODES) != NULL)
    wiringPiReturnCodes = TRUE ;

  if (wiringPiDebug)
    printf ("wiringPi: wiringPiSetupSys called\n") ;

  layout = piGpioLayout ();
  if (layout == SP_LAYOUT_F2S ||
      layout == SP_LAYOUT_F2P)
  {
     pinToGpio =  pinToGpioSP ;
    physToGpio = physToGpioSP ;
  }

// Open and scan the directory, looking for exported GPIOs, and pre-open
//	the 'value' interface to speed things up for later

  for (spin = 0 ; spin < 64 ; ++spin)
  {
    sprintf (fName, "/sys/class/gpio/P%d_%02d/value", spin/8, spin%8);
    pin = SPToGpio [spin] ;
    if (pin==-1) continue;
    sysFds [pin] = open (fName, O_RDWR) ;
  }

  initialiseEpoch () ;

  wiringPiMode = WPI_MODE_GPIO_SYS ;

  return 0 ;
}
