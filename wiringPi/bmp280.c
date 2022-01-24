/*
 * bmp280.c:
 *	Extend wiringPi with the BMP280 I2C/SPI Pressure and Temperature
 *	sensor. 
 *
 *	Information from the document held at:
 *		https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf
 *	was very useful when building this code.
 *
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

#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "wiringPi.h"
#include "wiringPiI2C.h"
#include "wiringPiSPI.h"
#include <sys/ioctl.h>

#include "bmp280.h"

#undef	DEBUG

#define	I2C_ADDRESS	    0x76
#define	BMP280_OSS	    0
#define I2C_RETRIES     0x0701
#define I2C_TIMEOUT     0x0702

// Static calibration data
//	The down-side of this is that there can only be one BMP280 in
//	a system - which is practice isn't an issue as it's I2C
//	address is fixed.

static uint16_t dig_t1, dig_p1;
static  int16_t dig_t2, dig_t3 ;
static  int16_t dig_p2, dig_p3, dig_p4, dig_p5, dig_p6, dig_p7, dig_p8, dig_p9;

// Pressure & Temp variables

uint32_t cPress, cTemp ;

static int altitude;

/*
 * writeByte:
 *	Write a byte to a register on BMP280 on the SPI bus.
 *********************************************************************************
 */

static void writeByte (int fd, uint8_t reg, uint8_t data, int bus)
{
  uint8_t spiData [4] ;

  if (bus==BMP280_SPI) 
  {
    spiData [0] = reg & 0x7f ;
    spiData [1] = data ;

    if (wiringPiSPIDataRW (fd, spiData, 2)<0 )
    {
        printf("wiringPiSPIDataRW failed.\n") ;
    }
  }
  else 
  {
    wiringPiI2CWriteReg8(fd, reg, data) ;
  }
}

/*
 * readByte:
 *	Read a byte from a register on the BMP280 on the SPI bus.
 *********************************************************************************
 */

static uint8_t readByte (int fd, uint8_t reg, int bus)
{
  uint8_t spiData [4] ;

  if (bus==BMP280_SPI) 
  {
    spiData [0] = reg | 0x80;

    if (wiringPiSPIDataRW (fd, spiData, 2)<0 )
    {
        printf("wiringPiSPIDataRW failed.\n");
    }
    return spiData [1] ;
  }
  else
  {
    return wiringPiI2CReadReg8 (fd, reg) ;
  }
}

/*
 * bmp280ReadTempPress:
 *	Does the hard work of reading the sensor
 *********************************************************************************
 */

int32_t t_fine;

int32_t bmp280_compensate_T_int32(int32_t adc_T)
{
	int32_t var1, var2, T;

	var1 = ((((adc_T>>3) - ((int32_t)dig_t1<<1))) * ((int32_t)dig_t2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)dig_t1)) * ((adc_T>>4) - ((int32_t)dig_t1)))>> 12) * ((int32_t)dig_t3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

uint32_t bmp280_compensate_P_int64(int32_t adc_P)
{
	int64_t var1, var2, p;

	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dig_p6;
	var2 = var2 + ((var1*(int64_t)dig_p5)<<17);
	var2 = var2 + (((int64_t)dig_p4)<<35);
	var1 = ((var1 * var1 * (int64_t)dig_p3)>>8) + ((var1 * (int64_t)dig_p2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_p1) >> 33;
	if (var1 == 0) {
		return 0; // avoid exception caused by division by zero
	}

	p = 1048576 - adc_P;
	p = (((p<<31)-var2)*3125) / var1;
	var1 = (((int64_t)dig_p9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)dig_p8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)dig_p7)<<4);

	return (uint32_t)p;
}

static void bmp280ReadTempPress (struct wiringPiNodeStruct *node)
{
  uint8_t data [4] ;
  int fd = node->fd ;
  int bus = node->bus ;

// Start a temperature sensor reading

  writeByte (fd, 0xF4, 0x2F, bus) ;
  delay(5);
  writeByte (fd, 0xF5, 0x10, bus) ;

// Read the raw data

  data [0] = readByte (fd, 0xFA, bus) ;
  data [1] = readByte (fd, 0xFB, bus) ;
  data [2] = readByte (fd, 0xFC, bus) ;

  cTemp = (data[0]<<16 | data[1]<<8 | data[2]) >> 4;

// And calculate...

  cTemp = bmp280_compensate_T_int32(cTemp);

// Read the raw data

  data [0] = readByte (fd, 0xF7, bus) ;
  data [1] = readByte (fd, 0xF8, bus) ;
  data [2] = readByte (fd, 0xF9, bus) ;
  
// And calculate...

  cPress = (data[0]<<16 | data[1]<<8 | data[2]) >> 4;
  cPress = bmp280_compensate_P_int64(cPress);
}


/*
 * myAnalogWrite:
 *	Write to a fake register to represent the height above sea level
 *	so that the peudo millibar register can read the pressure in mB
 *********************************************************************************
 */

static void myAnalogWrite (struct wiringPiNodeStruct *node, int pin, int value)
{
  int chan = pin - node->pinBase ;

  if (chan == 0)
    altitude = value ;
}

/*
 * myAnalogRead:
 *********************************************************************************
 */

static int myAnalogRead (struct wiringPiNodeStruct *node, int pin)
{
  int chan = pin - node->pinBase ;

  bmp280ReadTempPress (node) ;

  /**/ if (chan == 0)	// Read Temperature
    return cTemp ;
  else if (chan == 1)	// Pressure
    return cPress ;
  else
    return -9999 ;
}


void initBmp280(struct wiringPiNodeStruct *node) 
{
  int fd = node->fd;
  int bus = node->bus;
  // uint16_t ID;

// Read BMP280 ID

  // ID = readByte(fd, 0xd0, bus);
  // printf ("BMP280 ID = %xh\n", ID);

// Read calibration data

  dig_t1 = readByte (fd, 0x89, bus) << 8 | readByte (fd, 0x88, bus) ;
  dig_t2 = readByte (fd, 0x8B, bus) << 8 | readByte (fd, 0x8A, bus) ;
  dig_t3 = readByte (fd, 0x8D, bus) << 8 | readByte (fd, 0x8C, bus) ;
  dig_p1 = readByte (fd, 0x8F, bus) << 8 | readByte (fd, 0x8E, bus) ;
  dig_p2 = readByte (fd, 0x91, bus) << 8 | readByte (fd, 0x90, bus) ;
  dig_p3 = readByte (fd, 0x93, bus) << 8 | readByte (fd, 0x92, bus) ;
  dig_p4 = readByte (fd, 0x95, bus) << 8 | readByte (fd, 0x94, bus) ;
  dig_p5 = readByte (fd, 0x97, bus) << 8 | readByte (fd, 0x96, bus) ;
  dig_p6 = readByte (fd, 0x99, bus) << 8 | readByte (fd, 0x98, bus) ;
  dig_p7 = readByte (fd, 0x9B, bus) << 8 | readByte (fd, 0x9A, bus) ;
  dig_p8 = readByte (fd, 0x9D, bus) << 8 | readByte (fd, 0x9C, bus) ;
  dig_p9 = readByte (fd, 0x9F, bus) << 8 | readByte (fd, 0x9E, bus) ;
}

/*
 * bmp280i2cSetup:
 *	Create a new instance of a I2C GPIO interface. We know it
 *	has 4 pins, (4 analog inputs and 1 analog output which we'll shadow
 *	input 0) so all we need to know here is the I2C address and the
 *	user-defined pin base.
 *********************************************************************************
 */

int bmp280i2cSetup (const int pinBase)
{
  int fd ;
  struct wiringPiNodeStruct *node ;

  if ((fd = wiringPiI2CSetup (I2C_ADDRESS)) < 0)
    return FALSE ;

  node = wiringPiNewNode (pinBase, 4) ;

  node->fd          = fd ;
  node->bus         = BMP280_I2C ;
  node->analogRead  = myAnalogRead ;
  node->analogWrite = myAnalogWrite ;

	ioctl(fd, I2C_TIMEOUT, 1);  /*set timeout value*/
	ioctl(fd, I2C_RETRIES, 2);  /*set retry times*/

  initBmp280 (node);

  return TRUE ;
}

int bmp280spiSetup (const int pinBase, int spiChannel)
{
  struct wiringPiNodeStruct *node ;

  if (wiringPiSPISetup (spiChannel, 8000000) < 0)	// 10MHz Max
    return FALSE ;

  node = wiringPiNewNode (pinBase, 4) ;

  node->fd          = spiChannel ;
  node->bus         = BMP280_SPI ;
  node->analogRead  = myAnalogRead ;
  node->analogWrite = myAnalogWrite ;

  initBmp280 (node);

  return TRUE ;
}
