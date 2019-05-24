#include <avr/io.h>
#include <avr/interrupt.h>

#include <util/delay.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "lcd.h"
#include "pff.h"


FATFS fs;					// sistemul de fisiere


void IO_init(void)
{
	
	DDRA &= ~(1 << PA0);			// PA0 ca input, cu pull-up
	PORTA |= (1 << PA0);

	DDRA &= ~(1 << PA1);			// PA1 ca input, cu pull-up
	PORTA |= (1 << PA1);


	DDRB &= ~(1 << PB0);			// PB0 ca input, cu pull-up
	PORTB |= (1 << PB0);
	
 	DDRB &= ~(1 << PB0);			// PB0 ca input, cu pull-up
	PORTB |= (1 << PB0);

	DDRB &= ~(1 << PB1);			// PB1 ca input, cu pull-up
	PORTB |= (1 << PB1);
	
	DDRB &= ~(1 << PB2);			// PB2 ca input, cu pull-up
	PORTB |= (1 << PB2);

	DDRB &= ~(1 << PB3);			// PB3 ca input, cu pull-up
	PORTB |= (1 << PB3);

	DDRB &= ~(1 << PB4);			// PB4 ca input, cu pull-up
	PORTB |= (1 << PB4);
	
	DDRD &= ~(1 << PD0);                    // PD0 ca input, cu pull-up
	PORTD |= (1 << PD0);

	DDRD &= ~(1 << PD1);                    // PD1 ca input, cu pull-up
	PORTD |= (1 << PD1);

	DDRD &= ~(1 << PD5);                    // PD5 ca input, cu pull-up
	PORTD |= (1 << PD5);

	PCICR = (1 << PCIE0);		// enable interrupts for PORTA
	PCMSK0 = (1 << PCINT1) | (1 << PCINT0); // PA0, PA1

}

/*---------------------------------------------------------------------------*/
/* Ceas                                                                      */
/*---------------------------------------------------------------------------*/

volatile uint8_t hours = 0;
volatile uint8_t minutes = 0;
volatile uint8_t seconds = 0;

volatile int hold_pe_melodie = 0;
volatile int is_playing = 0;


void timer2_init(void)
{
	// set compare at each milisecond
	OCR2A = 124; //top
	// interrupt on compare A
	TIMSK2 |= (1 << OCIE2A); //enable la interrupt pe compare
	// CTC, top OCRA
	TCCR2A |= (1 << WGM21);
	// prescaler 128
	TCCR2B |= (1 << CS20) | (1 << CS22);
}


/*---------------------------------------------------------------------------*/
/* Player audio                                                              */
/*---------------------------------------------------------------------------*/

/*
 * Four-Character Code - folosit pentru a indentifica formate de date
 */
#define FCC(c1, c2, c3, c4) \
	(((DWORD)(c4) << 24) + \
	 ((DWORD)(c3) << 16) + \
	 (( WORD)(c2) <<  8) + \
	 (( BYTE)(c1) <<  0))


	 uint8_t	buf[2][256];	// wave output buffers (double buffering)
const	 uint16_t	buf_size = 256;	// front and back buffer sizes
volatile uint8_t	buf_front = 0;	// front buffer index (current buffer used)
volatile uint8_t	buf_pos = 0;	// current buffer position
volatile uint8_t	buf_sync = 0;

#define BUF_FRONT	(buf[buf_front])
#define BUF_BACK	(buf[1 - buf_front])


ISR(TIMER0_COMPA_vect) //baga in buffer urmatoarele note dintr-o melodie 
{
	OCR1B = BUF_FRONT[buf_pos++];

	// swap buffers when end is reached (end is 256 <=> overflow to 0)
	if(buf_pos == 0)
		buf_front = 1 - buf_front;
}

void timer0_start(void)
{
	// interrupt on compare A
	TIMSK0 |= (1 << OCIE0A);
	// CTC, top OCRA
	TCCR0B |= (0 << WGM02);
	TCCR0A |= (1 << WGM01) | (0 << WGM00);
	// prescaler 8
	TCCR0B |= (2 << CS00);
}

void timer0_stop(void)
{
	TCCR0B = 0;
	TCCR0A = 0;
	TIMSK0 = 0;
	OCR0A = 0;
	TCNT0 = 0;
}

void timer1_start(void)
{
	// 8-bit FastPWM
	TCCR1B |= (1 << WGM12);
	TCCR1A |= (1 << WGM10);
	// channel B inverted
	TCCR1A |= (1 << COM1B0) | (1 << COM1B1);
	// prescaler 1
	TCCR1B |= (1 << CS10);
}

void timer1_stop(void)
{
	TCCR1B = 0;
	TCCR1A = 0;
	OCR1B = 0;
	TCNT1 = 0;
}

bool continue_play()
{
	if(is_playing < 1)
		return false;

	return true;
}

/*
 * Incarca header-ul unui fisier WAVE
 *
 * @return DWORD
 * 	0 => format invalid
 * 	1 => eroare I/O
 * 	>1 => numarul de sample-uri
 */
DWORD load_header(void)
{
	DWORD size;
	WORD ret;

	// citeste header-ul (12 octeti)
	if(pf_read(BUF_FRONT, 12, &ret))
		return 1;

	if(ret != 12 || LD_DWORD(BUF_FRONT + 8) != FCC('W','A','V','E'))
		return 0;

	for(;;)
	{
		// citeste chunk ID si size
		pf_read(BUF_FRONT, 8, &ret);
		if(ret != 8)
			return 0;

		size = LD_DWORD(&BUF_FRONT[4]);

		// verifica FCC
		switch(LD_DWORD(&BUF_FRONT[0]))
		{
			// 'fmt ' chunk
			case FCC('f','m','t',' '):
				// verifica size
				if(size > 100 || size < 16) return 0;

				// citeste continutul
				pf_read(BUF_FRONT, size, &ret);
				// verifica codificarea
				if(ret != size || BUF_FRONT[0] != 1) return 0;
				// verifica numarul de canale
				if(BUF_FRONT[2] != 1 && BUF_FRONT[2] != 2) return 0;
				// verifica rezolutia
				if(BUF_FRONT[14] != 8 && BUF_FRONT[14] != 16) return 0;

				// seteaza sampling rate-ul
				//           F_CPU / prescaler / size
				OCR0A = (BYTE)(F_CPU / 8 / LD_WORD(&BUF_FRONT[4])) - 1;
				
				//OCR0A = (BYTE)(45) - 1;
				//OCR0A = (BYTE) (44099/256);
				//LCD_clear_top_line();
				/*char asd[10];
				snprintf(asd, sizeof(asd), "%ld", F_CPU / 22,67 / (unsigned int) size );
				LCD_printAt(0x00, asd);*/



				break;

			// 'data' chunk => incepe redarea
			case FCC('d','a','t','a'):
				return size;

			// 'LIST' chunk => skip
			case FCC('L','I','S','T'):
			// 'fact' chunk => skip
			case FCC('f','a','c','t'):
				pf_lseek(fs.fptr + size);
				break;

			// chunk necunoscut => eroare
			default:
				return 0;
		}
	}

	return 0;
}

/*
 * Functie care reda un fisier audio
 *
 * path - calea absoluta a fisierului
 *
 * @return UINT
 *	FR_OK daca a rulat cu succes fisierul
 */

void start_leds(const char *name) {
	if (strstr(name, "RE")) {
		PORTA |= (1 << PA5) | (1 << PA3);
		return;
	}
	if (strstr(name, "MI")) {
		PORTA |= (1 << PA4) ;
		return;
	}
	if (strstr(name, "FA")) {
		PORTA |= (1 << PA5) | (1 << PA4);
		return;
	}
	if (strstr(name, "SOL")) {
		PORTA |= (1 << PA4) | (1 << PA3);
		return;
	}
	if (strstr(name, "LA")) {
		PORTA |= (1 << PA5) ;
		return;
	}
	if (strstr(name, "SI")) {
		PORTA |= (1 << PA3) ;
		return;
	}

	PORTA |= (1 << PA5) | (1 <<PA3) | (1 << PA4);
}

void stop_leds() {
	PORTA &= ~(1 << PA3) & ~(1 << PA4) & ~(1 <<PA5);
}

UINT play(const char *path)
{
	FRESULT ret;

	if((ret = pf_open(path)) == FR_OK)
	{
		is_playing = 1;
		seconds = 0;
		minutes = 0;
		hours = 0;

		WORD bytes_read;

		// incarca header-ul fisierului
		DWORD current_size = load_header();
		if(current_size < buf_size)
			return FR_NO_FILE;

		// align to sector boundary
		ret = pf_lseek((fs.fptr + 511) & ~511);
		if(ret != FR_OK)
			return ret;

		// fill front buffer
		ret = pf_read(BUF_FRONT, buf_size, &bytes_read);
		if(ret != FR_OK)
			return ret;
		if(bytes_read < buf_size)
			return ret;

		// reset front buffer index
		buf_pos = 0;

		//start LEDs
		start_leds(path);

		// start output
		timer0_start();
		timer1_start();
		DDRD |= (1 << PD4);

		while(continue_play())
		{
			uint8_t old_buf_front = buf_front;

			// fill back buffer
			ret = pf_read(BUF_BACK, buf_size, &bytes_read);
			if(ret != FR_OK)
				break;
			if(bytes_read < buf_size)
				break;

			// wait for buffer swap
			while(old_buf_front == buf_front) ;
		}

		// stop output
		DDRD &= ~(1 << PD4);
		timer1_stop();
		timer0_stop();
		is_playing = 0;
		stop_leds();
	}

	return ret;
}


#define MUSIC "pm"

void get_music(int n, const char *folder, char *filename)
{
	DIR dir;
	FILINFO fil;

	char i = 0;

	pf_opendir(&dir, MUSIC);
	for(i = 0; i < n; i++) {
		pf_readdir(&dir, &fil);
		if(fil.fname[0] == 0x00) {
			filename[0] = 0x00;
			return;
		}
	}

	filename[0] = 0;
	strcat(filename, folder);
	strcat(filename, "/");
	strcat(filename, fil.fname);
}

char current_file_no = 1;
char filename[32] = { 0 };
int inainte = -1;

void inapoi_inainte()
{
	if(!(PINA & (1 << PA1))){
		hold_pe_melodie = 1;
		is_playing = 0;
		filename[0] = 0;
		if(inainte == 0) {
				get_music(current_file_no+2, MUSIC, filename);				
		}	
		else
			get_music(current_file_no, MUSIC, filename);
			
		if (filename[0] == 0x00){
			current_file_no = 1;
			get_music(current_file_no, MUSIC, filename);
		}

		LCD_clear_top_line();	
		LCD_printAt(0x00, filename);
		if(inainte == 1 || inainte == -1)
			current_file_no += 1;
		else {
			if (current_file_no == 1) 
				current_file_no += 1;
			else
				current_file_no += 3;
		}
		inainte = 1;
	}

	if(!(PINA & (1 << PA0))){
		hold_pe_melodie = 1;
		is_playing = 0;
		filename[0] = 0;
		if(inainte == 1)
			get_music(current_file_no-2, MUSIC, filename);
		else
			get_music(current_file_no, MUSIC, filename);
		if (filename[0] == 0x00){
			current_file_no = 8;
			get_music(current_file_no, MUSIC, filename);
		}

		LCD_clear_top_line();	
		LCD_printAt(0x00, filename);	
		
		if(current_file_no == 1)
				current_file_no = 8;
		else{
			if (inainte == 1)
				current_file_no -= 3;
			else
				current_file_no -= 1;
		}
		inainte = 0;
	}
		
}

ISR(TIMER2_COMPA_vect)
{
	static uint16_t miliseconds = 0;

/*
	if(++miliseconds != 1000)
		return;
	miliseconds = 0;
	
	if(++seconds == 60)
	{
		seconds = 0;

		if(++minutes == 60)
		{
			minutes = 0;

			if(++hours == 24)
				hours = 0;
		}
	}
*/
	if (hold_pe_melodie) {
		hold_pe_melodie++;
		miliseconds = 0;
	}

	if (is_playing) {

		if(++miliseconds < 1000)
			return;
		miliseconds = 0;
		
		if(++seconds == 60)
		{
			seconds = 0;

			if(++minutes == 60)
			{
				minutes = 0;

				if(++hours == 24)
					hours = 0;
			}
		}		


		char buf[7];
		snprintf(buf, sizeof(buf), "%02d:%02d", minutes, seconds);

		LCD_clear_bottom_line();
		LCD_printAt(0x40, buf);
	}


}

ISR(PCINT0_vect) {
	if(!(PINA & (1 << PA1))){
		hold_pe_melodie = 1;
		is_playing = 0;
		LCD_clear_bottom_line();
	}

	if(!(PINA & (1 << PA0))){
		hold_pe_melodie = 1;
		is_playing = 0;
		LCD_clear_bottom_line();
	}
}

void note_muzicale() {
	
	if(!(PINB & (1 << PB0))) {
		LCD_clear_top_line();	
		LCD_printAt(0x00, "DO Jos");
		play("pm/DO1.WAV");
	}

	if(!(PINB & (1 << PB1)))
	{
		LCD_clear_top_line();	
		LCD_printAt(0x00, "RE");
		play("pm/RE.WAV");
	}

	if(!(PINB & (1 << PB2)))
	{
		LCD_clear_top_line();	
		LCD_printAt(0x00, "MI");
		play("pm/MI.WAV");
	}

	if(!(PINB & (1 << PB3)))
	{
		LCD_clear_top_line();	
		LCD_printAt(0x00, "FA");
		play("pm/FA.WAV");
	}

	if(!(PINB & (1 << PB4)))
	{
		LCD_clear_top_line();	
		LCD_printAt(0x00, "SOL");
		play("pm/SOL.WAV");
	}

	if(!(PIND & (1 << PD5)))
	{
		LCD_clear_top_line();	
		LCD_printAt(0x00, "LA");
		play("pm/LA.WAV");
	}

	if(!(PIND & (1 << PD1)))
	{
		LCD_clear_top_line();	
		LCD_printAt(0x00, "SI");
		play("pm/SI.WAV");
	}

	if(!(PIND & (1 << PD0)))
	{	//timer2_init();
		LCD_clear_top_line();	
		LCD_printAt(0x00, "DO Sus");
		play("pm/DO2.WAV");
	}

}

int main (void)
{
	LCD_init();
	timer2_init();
	sei();

	for(;;)
	{
		// mount filesystem
		LCD_printAt(0x00, "trying...");

		if(pf_mount(&fs) != FR_OK)
		{
			// wait a while and retry
			_delay_ms(1000);
			continue;
		}

		IO_init();
		LCD_clear_top_line();
		LCD_printAt(0x00, "mounted!");
		_delay_ms(1000);
		LCD_clear_top_line();

		for(;;)
		{
			inapoi_inainte();
			note_muzicale();
			if (hold_pe_melodie > 3000) {
				get_music(current_file_no, MUSIC, filename);
				play(filename);
				hold_pe_melodie = 0;
			}
			_delay_ms(100);
		}
	}

	return 0;
}
