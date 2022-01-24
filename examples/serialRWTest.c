/*
 * serialRWTest.c:
 *	Test the read/write of serial port. 
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
#include <wiringPi.h>
#include <wiringSerial.h>

int main(void)
{
    int fd;
    int dnum = 0;
    int count = 0;
    int pre_count = 0;
    unsigned char chr[100];

    wiringPiSetup();                          
    fd = serialOpen("/dev/ttyS1", 115200);   

    printf("ttyS1 uart test:\n");            
    serialPrintf(fd, "Now you can test!\n");    

    while (1) {
        dnum = serialDataAvail(fd);            
        if(dnum > 0) {
            chr[count] = serialGetchar(fd);  
            count++;
        }
        else {
            delay(2);
        }

        if (pre_count!=count) {
            pre_count = count;
        }
        else {
            if (count) {
                chr[count]=0;
                printf ("%s", chr);
                serialPrintf(fd, "%s", chr);
                if (chr[0]=='\n') break;
                count = 0;
            }
        }
    }

    serialPrintf(fd, "close\n"); 
    serialClose(fd);                          
    return 0;
}

