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
  uint8_t raw[15];


  event void Boot.booted() {

    //post SDtest();
    
    call AMControl.start();
  }
  
  event void AMControl.startDone(error_t error) {
    
    if (error) {
      return;
    }
     if( call DiagMsg.record() ) {
      
      call DiagMsg.str("started");
      call DiagMsg.send();
    }

    call MagInit.init();
    //call Leds.led0On();
    error=call Magnetometer.runContinuousConversion();
    //error=call Magnetometer.runSingleConversion();
    if(error!=SUCCESS)
      call Leds.led0On();
    
  }
  
  event void AMControl.stopDone(error_t error) { }
  
  event void MagTimer.fired() {
    
    //call Magnetometer.enableBus();
    call Magnetometer.readData();
  }
  
    
  void sendData() {

    if( call DiagMsg.record() ) {
      
      call DiagMsg.int16s(mag, 3);
      call DiagMsg.hex8s(raw, 13);
      call DiagMsg.send();
    }
  }

  event void Magnetometer.readDone(int16_t* data, error_t error) {
    
    if (error) {
      return;
    }

    //call Magnetometer.convertRegistersToData(data, mag);
    memcpy(mag,data,6);
    call Magnetometer.getLastRead(raw);
    //call Magnetometer.disableBus();
    
    sendData();
    
    call Leds.led1Toggle();    
  }
	
  event void Magnetometer.setOutputRateDone(error_t success) { }
  event void Magnetometer.setGainDone(error_t success) { }
  event void Magnetometer.goToSleepDone(error_t success) { }
  event void Magnetometer.setIdleDone(error_t success) { }
  event void Magnetometer.runSingleConversionDone(error_t success) { 
    if(success!=SUCCESS)
      call Leds.led0Toggle();
    call MagTimer.startPeriodic(1000);   
  }
  event void Magnetometer.runContinuousConversionDone(error_t success) {
    if(success!=SUCCESS)
      call Leds.led0Toggle();
    call MagTimer.startPeriodic(1000);   
  }
}
