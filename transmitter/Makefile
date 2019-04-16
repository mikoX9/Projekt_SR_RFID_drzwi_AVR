

#	CC=xyz	->definiowanie zmiennej
#	$(CC)	->uzywanie zmiennej

MCU = atmega328p
MCU_ = m328p
F_CPU = 8000000

FILE1 		= rfm70
LOCATION1 	= rfm70


all:	main.hex
	rm -f main.o
	rm -f main.elf
	rm -f $(FILE1).o
	avrdude -p $(MCU_) -c usbasp -e -U flash:w:main.hex

.c.o:
	avr-gcc -Wall -Os -DF_CPU=$(F_CPU) -c -mmcu=$(MCU) $(LOCATION1)/$(FILE1).c -o $(FILE1).o
	avr-gcc -Wall -Os -DF_CPU=$(F_CPU) -c -mmcu=$(MCU) main.c -o main.o

main.elf: main.o $(FILE1).o
	avr-gcc -Wall -Os -DF_CPU=$(F_CPU) -mmcu=$(MCU) main.o $(FILE1).o -o main.elf

main.hex: main.elf
	rm -f main.hex
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex

clean:
	rm -f main.hex
	rm -f main.o
	rm -f main.elf
	rm -f $(FILE1).o




###################################################################