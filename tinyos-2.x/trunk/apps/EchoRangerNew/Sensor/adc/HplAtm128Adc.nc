/// $Id: HplAtm128Adc.nc,v 1.6 2010-06-29 22:07:43 scipio Exp $

/*
 * Copyright (c) 2004-2005 Crossbow Technology, Inc.  All rights reserved.
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
 * - Neither the name of Crossbow Technology nor the names of
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

#include "Atm128Adc.h"

/**
 * HPL interface to the Atmega128 A/D conversion subsystem. Please see the
 * Atmega128 manual for full details on the functioning of this subsystem.
 * <p>
 * A word of warning: the Atmega128 SLEEP instruction initiates an A/D
 * conversion when the ADC and ADC interrupt are enabled.
 *
 * @author Martin Turon <mturon@xbow.com>
 * @author Hu Siquan <husq@xbow.com>
 * @author David Gay
 */

interface HplAtm128Adc {
  
  /**
   * Sets the ADC channel
   * @param mux New channel
   */
  async command void setChannel(uint8_t mux);
  
  /**
   * Returns the current ADC channel
   * @return Current channel
   */
  async command uint8_t getChannel();
  
  /**
   * Turns ADLAR (ADC left adjust result) on/off
   * @param adlarOn TRUE turns on ADLAR, FALSE turns it off
   */
  async command void setAdlar(bool adlarOn);
  
  /**
   * Returns ADLAR (ADC left adjust result) status
   * @returns TRUE if ADLAR is on.
   */
  async command bool isAdlarOn();
  
  /**
   * Sets the reference voltage
   * @param ref New reference voltage
   */
  async command void setRef(uint8_t ref);
  
  /**
   * Returns the current reference voltage
   * @returns the current reference voltage
   */
  async command uint8_t getRef();

  /**
   * Sets the Track and hold time
   * @param time New track and hold time-1 in ADC clock cycles
   */
  async command void setTrackHoldTime(uint8_t time);
  
  /**
   * Returns the current track and hold time
   * @returns the current track and hold time-1 in ADC clock cycles
   */
  async command uint8_t getTrackHoldTime();

  /**
   * Sets the StartUp time
   * @param time New startup time (actual time is 4(time+1), min 20us)
   */
  async command void setStartUpTime(uint8_t time);
  
  /**
   * Returns the current startup time
   * @returns the current startup time (actual time is 4(time+1), min 20us)
   */
  async command uint8_t getStartUpTime();

  /**
   * Read the ADCSRA (ADC control) register
   * @return Current ADCSRA value
   */
  async command Atm128Adcsra_t getAdcsra();
  /**
   * Set the ADCSRA (ADC control) register
   * @param adcsra New ADCSRA value
   */
  async command void setAdcsra(Atm128Adcsra_t adcsra);

  /**
   * Read the latest A/D conversion result
   * @return A/D value
   */
  async command uint16_t getValue();

  /// A/D control utilities. All of these clear any pending A/D interrupt.

  /**
   * Enable ADC sampling
   */
  async command void enableAdc();
  /**
   * Disable ADC sampling
   */
  async command void disableAdc();

  /**
   * Enable ADC interrupt
   */
  async command void enableInterruption();
  /**
   * Disable ADC interrupt
   */
  async command void disableInterruption();
  /**
   * Clear the ADC interrupt flag
   */
  async command void resetInterrupt();

  /**
   * Start ADC conversion. If ADC interrupts are enabled, the dataReady event
   * will be signaled once (in non-continuous mode) or repeatedly (in
   * continuous mode).
   */
  async command void startConversion();
  /**
   * Enable continuous sampling
   */
  async command void setContinuous();
  /**
   * Disable continuous sampling
   */
  async command void setSingle();

  /* A/D status checks */

  /**
   * Is ADC enabled?
   * @return TRUE if the ADC is enabled, FALSE otherwise
   */
  async command bool isEnabled();
  /**
   * Is A/D conversion in progress?
   * @return TRUE if the A/D conversion is in progress, FALSE otherwise
   */
  async command bool isStarted();
  /**
   * Is A/D conversion complete? Note that this flag is automatically
   * cleared when an A/D interrupt occurs.
   * @return TRUE if the A/D conversion is complete, FALSE otherwise
   */
  async command bool isComplete();

  /**
   * Set ADC prescaler selection bits
   * @param scale New ADC prescaler. Must be one of the ATM128_ADC_PRESCALE_xxx
   *   values from Atm128Adc.h
   */
  async command void setPrescaler(uint8_t scale);

  /**
   * Cancel A/D conversion and any pending A/D interrupt. Also disables the
   * ADC interruption (otherwise a sample might start at the next sleep
   * instruction). This command can assume that the A/D converter is enabled. 
   * @return TRUE if an A/D conversion was in progress or an A/D interrupt
   *   was pending, FALSE otherwise. In single conversion mode, a return
   *   of TRUE implies that the dataReady event will not be signaled.
   */
  async command bool cancel();

  /**
   * A/D interrupt occured
   * @param data Latest A/D conversion result
   */
  async event void dataReady(uint16_t data);     
}
