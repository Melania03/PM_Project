#include <avr/io.h>
#include <util/delay.h>
#include "spi.h"

void SPI_init(){
	PORTB |= (1 << PB4);
	_delay_ms(1);
	SPI_DDR |= (1 << SPI_MOSI) | (1 << SPI_SCK);
	SPCR0 = (1 << SPE0) | (1 << MSTR0);
}

uint8_t SPI_exchange(uint8_t data){
	SPDR0 = data;
	while(!(SPSR0 & (1 << SPIF0)));
	return SPDR0;
}

