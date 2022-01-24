#include <wiringPi.h>
#include <stdio.h>
#include <bmp280.h>
#include <unistd.h>

int main (void)
{
    int temp, pres;
    int bus = BMP280_I2C;
    int bOK;

    wiringPiSetup () ;

    if (bus==BMP280_I2C)
        bOK = bmp280i2cSetup (64) ;
    else
    if (bus==BMP280_SPI)
        bOK = bmp280spiSetup (64, 0) ;

    if (bOK) {
        printf ("Setup failed.\n");
        return 1;
    }

    while (1) {
        temp = analogRead(64);
        pres = analogRead(65);
        printf ("temp = %.2f, press = %.2f\n", temp/100.0, pres/256.0);
        sleep(1);
    }
    return 0;
}

