#include <inttypes.h>
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "lcdpcf8574/lcdpcf8574.h"
#include "rfm70/rfm70.h"  
 
int main()
{

    sei();

    lcd_init(LCD_DISP_ON_BLINK);
    lcd_home();
    lcd_led(0); //set led

    lcd_clrscr();
//    lcd_puts("kurwa dziala");
  //  lcd_puts("kurwa dziala");



    uint8_t bufor[32];  
    initRFM(); //inicjalizacja RFM70
     
    setModeRX(); //tryb odbioru
    setChannel(8); // kanał 8

    DDRD |=(1<<PD1);
    PORTD |=(1<<PD1);

//lcd_puts("kurwa dziala");
    while(1)
    { 
        if (receivePayload(bufor)) // odbieramy dane i przesyłamy je dalej za pomocą uarta
        {
           // uart_puts((char*)bufor);
            //uart_puts("\n");
            //PORTD = (PORTD & (1<<PD0)) | (bufor[0]<<1);

           PORTD ^= (1<<PD1);
           lcd_puts((char*)bufor);
           //lcd_putc((char*)bufor);

        }  

    }
}