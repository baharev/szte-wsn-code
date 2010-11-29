#define F_CPU 16000000UL

#include <inttypes.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <avr/sleep.h>
#include <avr/power.h>

volatile uint8_t ison=0;

void init(){
	DDRB = 0xff;
	PORTB= 0;
	DDRD = 0xff;
	PORTD= 0;
	DDRE = 0xff;
	PORTE= 0;
	DDRF = 0xff;
	PORTF= 0;
}

ISR(TIMER2_OVF_vect) {
	ison = ~ison;
}

int main() {
	//DDRE |= _BV(PE3);
	//init();
	
	MCUCR |= (1 << JTD);
	asm volatile("nop"::);
	MCUCR |= (1 << JTD);
	
	//DDRF |= _BV(PF0);
	PORTF |= _BV(PF0);
	
	
	TRXPR = 1 << SLPTR;
	//DRTRAM0 |= (1 << ENDRT);
	PRR0 = (1 << PRTWI) | (1 << PRTIM0) | (1 << PRPGA) | (1 << PRTIM1) | (1 << PRSPI) | (1 << PRUSART0) | (1 << PRADC);
	PRR1 = (1 << PRTRX24) | (1 << PRTIM5) | (1 << PRTIM4) | (1 << PRTIM3) | (1 << PRUSART1);
	//PRR2 = (1 << PRRAM3) | (1 << PRRAM2) | (1<< PRRAM1) | (1 << PRRAM0);
	
	
	TCCR2B = (1 << CS20) | (1<< CS21) | (1<< CS22);
	TIMSK2 = 1 << TOIE2;
	ASSR   = 1 << AS2;
	
	sei();
	set_sleep_mode(SLEEP_MODE_PWR_SAVE);
	for(;;) {
		//TRXPR = 1 << SLPTR;
		sleep_enable();
		sleep_cpu();
		sleep_disable();
		OCR2B = ison;
		
		if(!ison) {
			//PORTE = _BV(PE3);
		}
		else{
			//PORTE = 0;
		}
		
		while ((ASSR & (1<<OCR2BUB)) != 0) {}	
	}
}
