/*
lcdpcf8574 lib sample

copyright (c) Davide Gironi, 2013

Released under GPLv3.
Please refer to LICENSE file for licensing information.
*/


#include <avr/io.h>
#include <util/delay.h>

#include "lcdpcf8574/lcdpcf8574.h"


int main(void)
{
    lcd_init(LCD_DISP_ON_BLINK);

    //lcd go home
    lcd_home();

    uint8_t led = 0;
    lcd_led(led); //set led



char cos[6] = {'a','f','d','a','f','\0'};
char cos2[] = "drugi";

    while(1) {
    lcd_gotoxy(1, 3);
    //lcd_puts("kurwa dziadla");
    //lcd_data( (uint8_t)20 );
    lcd_puts( cos2 );
    _delay_ms(1000);
    }
}


