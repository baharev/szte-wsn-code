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

#include "ShimmerAdc.h"
#include "Mma_Accel.h"

module AppP {

  uses {
    
    interface Boot;
    interface Leds;
    
    interface Timer<TMilli> as MagTimer;
    interface Timer<TMilli> as AccelTimer;  
    interface Timer<TMilli> as SleepTimer;
    
    interface SplitControl as AMControl;
    interface AMSend;
    
    interface ShimmerAdc;

    interface Init as AccelInit;
    interface Mma_Accel;
    
    interface Init as MagInit;
    interface Magnetometer;    
  }
}

implementation {

  uint8_t channels[] = { 
    SHIMMER_ADC_ACCEL_X, 
    SHIMMER_ADC_ACCEL_Y,
    SHIMMER_ADC_ACCEL_Z,
    SHIMMER_ADC_TEMP,
  };

  enum {
    CHANNEL_COUNT      = 4,
    ACCEL_SAMPLE_COUNT = 20,
    MAG_SAMPLE_COUNT   = 4,
    MAG_SAMPLING_RATE  = 110,
    ACCEL_SAMPLING_RATE= 5,
    SLEEP_TIME         = 300
  };

  uint8_t n_sample = 0;
  
  typedef struct {
    uint32_t timestamp;
    int32_t  mag[3];
    uint32_t acc[3];
    uint32_t temp;
  } sample_t;
  
  sample_t sample;
  
  message_t msg;
  
  event void Boot.booted() {
    
    call Leds.led2On(); 
    
    call AMControl.start();
  }
  
  void magnetometer_sensing();
  
  event void AMControl.startDone(error_t error) {
    
    if (error) {

      return;
    }
    
    call Leds.led2Off();
    
    error = call ShimmerAdc.setChannels(channels, CHANNEL_COUNT);
    
    if (error) {
      call Leds.led0On();
      return;
    }
    
    call MagInit.init();    
    call Magnetometer.runContinuousConversion();
    
    call AccelInit.init();
    call Mma_Accel.setSensitivity(RANGE_1_5G);
    call Mma_Accel.wake(TRUE);    
 
  }
  
  void reset() {
    
    uint8_t i;
    
    n_sample = 0;

    for (i=0; i<3; ++i) {
      sample.mag[i] = 0;
      sample.acc[i] = 0;
    }
    
    sample.temp = 0;
  }
  
  void magnetometer_sensing() {
    
    reset();
    
    call Magnetometer.enableBus();
    
    call Magnetometer.readData();
  }

  event void Magnetometer.readDone(uint8_t* data, error_t error) {
    
    uint8_t i;
    int16_t mag[3];
    
    if (error) {
      call Leds.led0On();
      call MagTimer.startOneShot(MAG_SAMPLING_RATE);
      return;
    }

    call Magnetometer.convertRegistersToData(data, mag);
    
    for (i=0; i<3; ++i) {
      
      sample.mag[i] += mag[i];
    }
    
    if (++n_sample==MAG_SAMPLE_COUNT) {
      
      n_sample = 0;
      
      //sample.mag[0] = -10;
      //sample.mag[1] = 2;
      //sample.mag[2] = 3000;      
      
      //call Magnetometer.disableBus();
      
      call ShimmerAdc.sample();
    }
    else {
      
      call MagTimer.startOneShot(MAG_SAMPLING_RATE);
    }
  }
    
  event void MagTimer.fired() {
    
    call Magnetometer.readData();
  }
  
  void sendData();
  
  event void ShimmerAdc.sampleDone(uint32_t timestamp, uint16_t *data) {
    
    uint8_t i;
    
    for (i=0; i<3; ++i) {
      
      sample.acc[i] += data[i];
    }
    
    sample.temp = data[3];
    
    if (++n_sample==ACCEL_SAMPLE_COUNT) {
      
      sample.timestamp = timestamp;
      
      sendData();
    }
    else {
      
      call AccelTimer.startOneShot(ACCEL_SAMPLING_RATE);
    }
  }
  
  event void AccelTimer.fired() {
    
    call ShimmerAdc.sample();
  }
  
  void sendData() {
    
    error_t error;
    
    sample_t* pkt = (sample_t*) call AMSend.getPayload(&msg, sizeof(sample_t));
    
    if (pkt == NULL) {
      call Leds.led0On();
      return;
    }
    
    memcpy(pkt, &sample, sizeof(sample_t));

    // TODO Send to the basestation only
    error = call AMSend.send(AM_BROADCAST_ADDR, &msg, sizeof(sample_t));
    
    if (error != SUCCESS) {
      
      call SleepTimer.startOneShot(SLEEP_TIME);
    }
  }
  
  event void AMSend.sendDone(message_t* , error_t error) {
  
    if (error) {
      
      call Leds.led0On(); 
    }
    else {
           
      call Leds.led1Toggle(); 
    }
    
    call SleepTimer.startOneShot(SLEEP_TIME);
  }
  
  event void SleepTimer.fired() {
    
    magnetometer_sensing();
  }

  event void Magnetometer.writeDone(error_t success) {
          
    magnetometer_sensing();
  }
    
  event void AMControl.stopDone(error_t error) { }  

}
