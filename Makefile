PORT ?= /dev/ttyUSB0

all: lab4.hex

lab4.elf: lab4.c lcd.c pff.c sd.c spi.c
	avr-g++ -Wall -Wextra -mmcu=atmega324a -DF_CPU=16000000 -Os -o $@ $^

%.o: %.c
	avr-g++ -Wall -Wextra -mmcu=atmega324a -DF_CPU=16000000 -Os -o $@ -c $^ 

lab4.hex: lab4.elf
	avr-objcopy -j .text -j .data -O ihex $^ $@
	avr-size $^

clean:
	rm -rf lab4.hex lab4.elf

upload: lab4.hex
	avrdude -c arduino -P $(PORT) -b 57600 -p atmega324p -U flash:w:$<:a

.PHONY: all clean upload
