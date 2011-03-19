#define F_CPU 16000000UL

#include <inttypes.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
void init() {
	//spi+flash
	DDRB|=1<<PB1|1<<PB2|1<<PB4|1<<PB0;	
	PORTB|=1<<PB4;
	SPCR = (1<<SPE)|(1<<MSTR);
	//leds
	DDRE=1<<PE3|1<<PE5|1<<PE6|1<<PE7;
	ledset(0);
}

void transmit(char cData)

{

    /* Start transmission */

    SPDR = cData;

    /* Wait for transmission complete */

    while(!(SPSR & (1<<SPIF)));

}

void ledset(char val){
  if(val&1)
    PORTE|=1<<PE5;
  else
    PORTE&=~(1<<PE5);
  if(val&2)
    PORTE|=1<<PE6;
  else
    PORTE&=~(1<<PE6);
  if(val&4)
    PORTE|=1<<PE7;
  else
    PORTE&=~(1<<PE7);
  if(val&8)
    PORTE|=1<<PE3;
  else
    PORTE&=~(1<<PE3);
}

int main(void){

  init();
  ledset(0xff);
  _delay_ms(100);
  ledset(9);
  _delay_ms(100);
  wdt_enable(1);
  while(1);
  return 0;
}
