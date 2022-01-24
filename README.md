WiringPi for SP7021
=================================

This is modified from [unofficial mirror/fork of wiringPi](https://github.com/WiringPi/WiringPi) to support [SP7021 board (Sunplus evaluation board)](https://sunplus.atlassian.net/wiki/spaces/doc/pages/737444005/SP7021+Quick+Start). Because of SP7021 hardware is different from Raspberry Pi so that there are some hardware-related function would not  be supported. (like edge/pad drive/pwm-bal/pwm-ms ...)

Compile
-------
Set your cross compile from build script file then <pre>./build debian</pre> Copy deb file from "debian-template/wiringpi-2.60-1.deb" to SP7021 and install deb package <pre>sudo dpkg -i wiringpi-2.60-1.deb</pre>

Usage
-------
> GPIO pin mode setting
* gpio mode <pre>gpio mode `<pin>` `<mode>`
`<pin>:`
&emsp;wiringPi pin number
`<mode>:`
&emsp;in/input     : set pin as input
&emsp;out/output   : set pin as output
&emsp;pwm(0/1/2/3) : set pin as pwm 0,1,2,3 
&emsp;od           : set pin as open drain 
&emsp;in_inv       : set pin as input invert
&emsp;out_inv      : set pin as output invert
example: gpio mode 0 pwm0
</pre>

> GPIO pin read / write  
* gpio read <pre>gpio read `<pin>`</pre>
* gpio write <pre>gpio write `<pin>` `<value>`</pre>

> GPIO pwm
* set gpio pwm clock <pre>gpio pwmc `<pin>` `<clock>`
`<clock>:` set pwm clock (1~65535)
example: 
gpio pwmc 0 100 /* pwm clock = 100 Hz */
</pre>

* set gpio pwm duty <pre>gpio pwm `<pin>` `<value>`
`<value>:` set pwm duty ratio (1~255)
example: 
gpio pwm 0 25 /* duty ratio = 25/255 = 9.8% */
</pre>

> GPIO wait for interrupt  
* gpio wfi <pre>gpio wfi `<pin>` `<rising/falling/both>`
example: gpio wfi 0 rising
<br/>note: this function is implemented by polling register, not hardware interrupt.
</pre>

> GPIO extension
* gpio -x <pre>gpio -x extension:params `<read/write/aread/awrite>` `<pin>` 
example: 
gpio -x bmp280:64:spi:0 aread 64 `/*` read data from spi 0 `*/`
gpio -x bmp280:64:i2c aread 64 `/*` read data from i2c `*/`
</pre>
