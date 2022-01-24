/*
 * readall.c:
 *	The readall functions - getting a bit big, so split them out.
 *	Copyright (c) 2012-2018 Gordon Henderson
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


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <wiringPi.h>

extern int wpMode ;

#ifndef TRUE
#  define       TRUE    (1==1)
#  define       FALSE   (1==2)
#endif

/*
 * doReadallExternal:
 *	A relatively crude way to read the pins on an external device.
 *	We don't know the input/output mode of pins, but we can tell
 *	if it's an analog pin or a digital one...
 *********************************************************************************
 */

static void doReadallExternal (void)
{
  int pin ;

  printf ("+------+---------+--------+\n") ;
  printf ("|  Pin | Digital | Analog |\n") ;
  printf ("+------+---------+--------+\n") ;

  for (pin = wiringPiNodes->pinBase ; pin <= wiringPiNodes->pinMax ; ++pin)
    printf ("| %4d |  %4d   |  %4d  |\n", pin, digitalRead (pin), analogRead (pin)) ;

  printf ("+------+---------+--------+\n") ;
}


/*
 * doReadall:
 *	Read all the GPIO pins
 *	We also want to use this to read the state of pins on an externally
 *	connected device, so we need to do some fiddling with the internal
 *	wiringPi node structures - since the gpio command can only use
 *	one external device at a time, we'll use that to our advantage...
 *********************************************************************************
 */

static char *alts [] =
{
  "IN",
  "OUT",
  "PWM0",
  "PWM1",
  "PWM2",
  "PWM3",
  "OPENDRAIN",
  "IN_INV",
  "OUT_INV",
  "I2C0_CLK",  // 9
  "I2C0_DAT",
  "I2C1_CLK",
  "I2C1_DAT",
  "I2C2_CLK",
  "I2C2_DAT",
  "I2C3_CLK",
  "I2C3_DAT",

  "SPI0M_INT",  
  "SPI0M_CLK",
  "SPI0M_EN",
  "SPI0M_DO",
  "SPI0M_DI",
  "SPI1M_INT",
  "SPI1M_CLK",
  "SPI1M_EN",
  "SPI1M_DO",
  "SPI1M_DI",
  "SPI2M_INT",
  "SPI2M_CLK",
  "SPI2M_EN",
  "SPI2M_DO",
  "SPI2M_DI",
  "SPI3M_INT",
  "SPI3M_CLK",
  "SPI3M_EN",
  "SPI3M_DO",
  "SPI3M_DI",

  "SPI0S_INT",  
  "SPI0S_CLK",
  "SPI0S_EN",
  "SPI0S_DO",
  "SPI0S_DI",
  "SPI1S_INT",
  "SPI1S_CLK",
  "SPI1S_EN",
  "SPI1S_DO",
  "SPI1S_DI",
  "SPI2S_INT",
  "SPI2S_CLK",
  "SPI2S_EN",
  "SPI2S_DO",
  "SPI2S_DI",
  "SPI3S_INT",
  "SPI3S_CLK",
  "SPI3S_EN",
  "SPI3S_DO",
  "SPI3S_DI",

  "UART1_TX",  
  "UART1_RX",
  "UART1_CTS",
  "UART1_RTS",
  "UART2_TX",
  "UART2_RX",
  "UART2_CTS",
  "UART2_RTS",
  "UART3_TX",
  "UART3_RX",
  "UART3_CTS",
  "UART3_RTS",
  "UART4_TX",
  "UART4_RX",
  "UART4_CTS",
  "UART4_RTS",
} ;

static int physToWpi [64] =
{
  -1,           // 0
  -1, -1,       // 1, 2
   8, -1,
   9, -1,
   7, 15,
  -1, 16,
   0,  1,
   2, -1,
   3,  4,
  -1,  5,
  12, -1,
  13,  6,
  14, 10,
  -1, 11,       // 25, 26
  30, 31,	      
  21, -1,
  22, 26,
  23, -1,
  24, 27,
  25, 28,
  -1, 29,
  -1, -1,
  -1, -1,
  -1, -1,
  -1, -1,
  -1, -1,
  17, 18,
  19, 20,
  -1, -1, -1, -1, -1, -1, -1, -1, -1
} ;

static char *physNames [64] =
{
  NULL,

  "   3.3v", "5v     ",
  "GPIO. 8", "5v     ",
  "GPIO. 9", "0v     ",
  "GPIO. 7", "GPIO.15",
  "     0v", "GPIO.16",
  "GPIO. 0", "GPIO. 1",
  "GPIO. 2", "0v     ",
  "GPIO. 3", "GPIO. 4",
  "   3.3v", "GPIO. 5",
  "GPIO.12", "0v     ",
  "GPIO.13", "GPIO. 6",
  "GPIO.14", "GPIO.10",
  "     0v", "GPIO.11",
  "GPIO.30", "GPIO.31",
  "GPIO.21", "0v     ",
  "GPIO.22", "GPIO.26",
  "GPIO.23", "0v     ",
  "GPIO.24", "GPIO.27",
  "GPIO.25", "GPIO.28",
  "     0v", "GPIO.29",
       NULL, NULL,
       NULL, NULL,
       NULL, NULL,
       NULL, NULL,
       NULL, NULL,
  "GPIO.17", "GPIO.18",
  "GPIO.19", "GPIO.20",
   NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
} ;

/*
 * readallPhys:
 *	Given a physical pin output the data on it and the next pin:
 *| SP | wPi |   Name  |   Mode   | Val| Physical |Val |   Mode   | Name    | wPi | SP |
 *********************************************************************************
 */

static void readallPhys (int physPin)
{
  int pin ;

  //field 1
  if (physPinToGpio (physPin) == -1)
    printf (" |     |    ") ;
  else
    printf (" | %3d | %3d", physPinToGpio (physPin), physToWpi [physPin]) ;

  //field 2
  printf (" | %7s", physNames [physPin]) ;

  //field 3/4
  if (physToWpi [physPin] == -1)
    printf (" |%10s |  ", " ") ;
  else
  {
    /**/ if (wpMode == WPI_MODE_GPIO)
      pin = physPinToGpio (physPin) ;
    else if (wpMode == WPI_MODE_PHYS)
      pin = physPin ;
    else
      pin = physToWpi [physPin] ;

    printf (" |%10s", alts [getAlt (pin)]) ;
    printf (" | %d", digitalRead (pin)) ;
  }

  //field 5
  // Pin numbers:

  printf (" | %2d", physPin) ;
  ++physPin ;
  printf (" || %-2d", physPin) ;

  // Same, reversed

  //field 3/4

  if (physToWpi [physPin] == -1)
    printf (" |%2s | %8s ", " ", " ") ;
  else
  {
    /**/ if (wpMode == WPI_MODE_GPIO)
      pin = physPinToGpio (physPin) ;
    else if (wpMode == WPI_MODE_PHYS)
      pin = physPin ;
    else
      pin = physToWpi [physPin] ;

    printf (" | %d", digitalRead (pin)) ;
    printf (" | %-9s", alts [getAlt (pin)]) ;
  }

  // field 2
  printf (" | %-6s", physNames [physPin]) ;

  // field 1
  if (physToWpi     [physPin] == -1)
    printf (" |%5s|    ", " ") ;
  else
    printf (" | %-3d | %-3d", physToWpi [physPin], physPinToGpio (physPin)) ;

  printf (" |\n") ;
}


/*
 * allReadall:
 *	Read all the pins regardless of the model. Primarily of use for
 *	the compute module, but handy for other fiddling...
 *********************************************************************************
 */

static void allReadall (void)
{
  int pin ;

  printf ("+-----+------+-------+      +-----+------+-------+\n") ;
  printf ("| Pin | Mode | Value |      | Pin | Mode | Value |\n") ;
  printf ("+-----+------+-------+      +-----+------+-------+\n") ;

  for (pin = 0 ; pin < 27 ; ++pin)
  {
    printf ("| %3d ", pin) ;
    printf ("| %-4s ", alts [getAlt (pin)]) ;
    printf ("| %s  ", digitalRead (pin) == HIGH ? "High" : "Low ") ;
    printf ("|      ") ;
    printf ("| %3d ", pin + 27) ;
    printf ("| %-4s ", alts [getAlt (pin + 27)]) ;
    printf ("| %s  ", digitalRead (pin + 27) == HIGH ? "High" : "Low ") ;
    printf ("|\n") ;
  }

  printf ("+-----+------+-------+      +-----+------+-------+\n") ;

}

/*
 * piPlusReadall:
 *	Read all the pins on the model A+ or the B+ or actually, all 40-pin Pi's
 *********************************************************************************
 */

static void plus2header (int model)
{
  if (model == SP_MODEL_F2S)
    printf (" +-----+-----+---------+-----------+---+--SP F2S--+---+-----------+---------+-----+-----+\n") ;
  else if (model == SP_MODEL_F2P)
    printf (" +-----+-----+---------+-----------+---+--SP F2P--+---+-----------+---------+-----+-----+\n") ;
  else
    printf (" +-----+-----+---------+-----------+---+--SP  ? --+---+-----------+---------+-----+-----+\n") ;
}

static void spReadall (int model)
{
  int pin ;

  plus2header (model) ;

  printf (" | SP  | wPi |   Name  |   Mode    | V | Physical | V |   Mode    | Name    | wPi | SP  |\n") ;
  printf (" +-----+-----+---------+-----------+---+----------+---+-----------+---------+-----+-----+\n") ;
  for (pin = 1 ; pin <= 40 ; pin += 2)
    readallPhys (pin) ;
  printf (" +-----+-----+---------+-----------+---+----------+---+-----------+---------+-----+-----+\n") ;
  printf (" | SP  | wPi |   Name  |   Mode    | V | Physical | V |   Mode    | Name    | wPi | SP  |\n") ;

  plus2header (model) ;
}


/*
 * doReadall:
 *	Generic read all pins called from main program. Works out the Pi type
 *	and calls the appropriate function.
 *********************************************************************************
 */

void doReadall (void)
{
  int model, rev, mem, maker, overVolted ;

  if (wiringPiNodes != NULL)	// External readall
  {
    doReadallExternal () ;
    return ;
  }

  piBoardId (&model, &rev, &mem, &maker, &overVolted) ;

  if ((model == SP_MODEL_F2S) || (model == SP_MODEL_F2P) )
    spReadall (model) ;
  else
    printf ("Oops - unable to determine board type... model: %d\n", model) ;
}


/*
 * doAllReadall:
 *	Force reading of all pins regardless of Pi model
 *********************************************************************************
 */

void doAllReadall (void)
{
  allReadall () ;
}


/*
 * doQmode:
 *	Query mode on a pin
 *********************************************************************************
 */

void doQmode (int argc, char *argv [])
{
  int pin ;

  if (argc != 3)
  {
    fprintf (stderr, "Usage: %s qmode pin\n", argv [0]) ;
    exit (EXIT_FAILURE) ;
  }

  pin = atoi (argv [2]) ;
  printf ("%s\n", alts [getAlt (pin)]) ;
}
