//$Id: HilTimerMilliC.nc,v 1.3 2010-08-02 10:12:41 mmaroti Exp $

/* Copyright (c) 2000-2003 The Regents of the University of California.  
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the copyright holder nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * Millisecond timer for the mica family (see TEP102). The "millisecond"
 * timer system is built on hardware timer 0, running at 1024Hz.
 *
 * @author Cory Sharp <cssharp@eecs.berkeley.edu>
 * @author Martin Turon <mturon@xbow.com>
 */

#include "Timer.h"

configuration HilTimerMilliC {
  provides interface Init;
  provides interface Timer<TMilli> as TimerMilli[uint8_t num];
  provides interface LocalTime<TMilli>;
}
implementation
{
	enum
	{
		TIMER_COUNT = uniqueCount(UQ_TIMER_MILLI)
	};

	components new VirtualizeTimerC(TMilli, TIMER_COUNT);
	TimerMilli = VirtualizeTimerC;
	VirtualizeTimerC.TimerFrom -> AlarmToTimerC;

	components new AlarmToTimerC(TMilli);
	AlarmToTimerC.Alarm -> AlarmCounterMilliP;

	components new CounterToLocalTimeC(TMilli);
	LocalTime = CounterToLocalTimeC;
	CounterToLocalTimeC.Counter -> AlarmCounterMilliP;

#ifdef TIMER_AT_32KHZ

	components new TransformAlarmCounterC(TMilli, uint32_t, T32khz, uint32_t, 5, uint8_t) as AlarmCounterMilliP;
	AlarmCounterMilliP.AlarmFrom -> AlarmCounter32khzP;
	AlarmCounterMilliP.CounterFrom -> AlarmCounter32khzP;

	components AlarmCounter32khzP;
	Init = AlarmCounter32khzP;

#else

	components AlarmCounterMilliP;
	Init = AlarmCounterMilliP;

#endif
}
