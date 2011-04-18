/** Copyright (c) 2011, University of Szeged
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
* Author: Ali Baharev
*/

module AppP {

  uses {
    
    interface Boot;
    interface Leds;
    
    interface StdControl as SDStdControl;
    interface SD;
    
    interface Init as MagInit;
    interface Magnetometer;
    interface Timer<TMilli> as MagTimer;
    
    interface SplitControl as AMControl;
    interface DiagMsg;
  }
}

implementation {

  uint8_t buffer[512];
  int16_t mag[3];

  task void SDtest() {

    error_t error;
    
    uint32_t size;

    call Leds.led1On();

    error = call SDStdControl.start();

    if( error != SUCCESS ) {
      call Leds.led0On();
      post SDtest();
      return;
    }

    error = call SD.readBlock(0, buffer);

    if (error) {
      call Leds.led0On();
      post SDtest();
      return;
    }

    call Leds.led0Off();
    call Leds.led1Off();
  }
  
  async event void SD.available() {

    //call Leds.led2On();
  }

  async event void SD.unavailable() { }

  event void Boot.booted() {

    //post SDtest();
    
    call AMControl.start();
  }
  
  event void AMControl.startDone(error_t error) {
    
    if (error) {
      call Leds.led0On();
      return;
    }

    call MagInit.init();
    
    call Magnetometer.runContinuousConversion();
    
    call MagTimer.startPeriodic(1000);   
  }
  
  event void AMControl.stopDone(error_t error) { }
  
  event void MagTimer.fired() {
    
    call Magnetometer.enableBus();
    
    call Magnetometer.readData();
  }
  
    
  void sendData() {

    if( call DiagMsg.record() ) {
      
      call DiagMsg.int16s(mag, 3);
      call DiagMsg.send();
    }
  }

  event void Magnetometer.readDone(uint8_t* data, error_t error) {
    
    if (error) {
      call Leds.led0On();
      return;
    }

    call Magnetometer.convertRegistersToData(data, mag);
    
    call Magnetometer.disableBus();
    
    sendData();
    
    call Leds.led1Toggle();    
  }
	
  event void Magnetometer.writeDone(error_t success) { }

}
