#include <inttypes.h>
#include <avr/io.h>
#include <util/delay.h>
 
#include "rfm70/rfm70.h"
 
int main()
{
        DDRD |=(1<<PD1);
        PORTD |=(1<<PD1);



    char test[]={'1', 'a', '5', 't','y'};    
    uint8_t test2[]={'2', 'A', 's', 't','y'};   
    initRFM(); //inicjalizacja RFM70
     



    setModeTX(); //tryb nadawania
    setChannel(8); // kanał 8
    setPower(3); // maksymalna moc (0: -10dBm | 1: -5dBm | 2: 0dBm | 3: 5dBm)
   
               PORTD &=~(1<<PD1);

    while (1)
    {   
        sendPayload(test, 2, 0); //tablica, dlugosc, 0 - bez potwierdzenia | 1 - z potwierdzeniem
        _delay_ms(1000);  
        sendPayload(test2, 2, 0); //tablica, dlugosc, 0 - bez potwierdzenia | 1 - z potwierdzeniem
        _delay_ms(1000);  
    
 /*       PORTD = 0xff;
        _delay_ms(1000);
        PORTD = 0x00;
        _delay_ms(1000);
*/
    }
}