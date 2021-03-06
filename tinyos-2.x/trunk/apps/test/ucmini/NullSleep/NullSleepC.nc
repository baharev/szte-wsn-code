#include <avr/power.h>

/*
* Copyright (c) 2010, University of Szeged
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* - Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* - Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* - Neither the name of University of Szeged nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
* OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Author: Zsolt Szabo
*/

#include "Stm25p.h"
module NullSleepC @safe()
{
  uses interface Boot;
	uses interface Leds;
	uses interface GeneralIO as VoltMeter;
	uses interface GeneralIO as SSN;
//	uses interface AtmegaCompare<uint32_t>;

	uses interface Alarm<T62khz, uint32_t>;
}
implementation
{
volatile uint8_t ison=0;
volatile uint16_t timer = 0;

	void init(void) {
		call VoltMeter.set();
		call SSN.makeOutput();
		call SSN.clr();
	}

  event void Boot.booted() {
		init();
		
		//call SpiRes.request();
    //call Stm25pSpi.powerDown();
		//call SpiByte.write(0xB9);
//		call AtmegaCompare.set(0x00ff);
		SCCR0 = (1 << SCEN);
		//power_all_disable();

//		call AtmegaCompare.start();
call Alarm.start(0x0100);

  }//booted

	async event void Alarm.fired() {
		call Alarm.stop();
		ison = ~ison;
		//ison?(call Leds.led3On()):(call Leds.led3Off());
		call Alarm.start(0xFF00);
	}
}

