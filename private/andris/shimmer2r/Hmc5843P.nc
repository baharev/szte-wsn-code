/*
 * Copyright (c) 2011, Shimmer Research, Ltd.
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:

 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Shimmer Research, Ltd. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * @author Steve Ayer
 * @date February, 2011
 *
 * broken out from original gyromagboard* interface/implementation
 *
 * this module adds functionality of honeywell hmc5843 magnetometer
 * to similar gyro board (idg-500 3-axis plus user button and led)
 *
 * since gyro board is stand-alone, this module uses existing gyro module
 * and interface for everything but magnetometer.
 */

#include "Magnetometer.h"

module Hmc5843P {
  provides {
    interface Init;
    interface Magnetometer;
  }
  uses {
    interface I2CPacket<TI2CBasicAddr> as I2CPacket;
    interface Resource;
    interface DiagMsg;
  }
}

implementation {
//   extern int sprintf(char *str, const char *format, ...) __attribute__ ((C));
//   extern int snprintf(char *str, size_t len, const char *format, ...) __attribute__ ((C));

  enum{
    S_OFF=0,
    S_GOSLEEP,
    S_GOIDLE,
    S_SETGAIN,
    S_SETGAIN_CONTMEAS,
    S_SETGAIN_SLEEP,
    S_SETRATE,
    S_SETRATE_CONTMEAS,
    S_SETRATE_SLEEP,
    S_RUN_SINGLE,
    S_RUN_CONT_MEAS,
    S_READ,
    S_READ_CONTMEAS,
    S_READ_SLEEP,
    
    //if(state<64) we couldn't receive commands
    S_IDLE=65,
    S_SLEEP=66,
    S_CONTMEAS=67,
  };
  
  enum{
    READLENGTH=10,
    WRITELENGTH=2,
    READONCE=READLENGTH,
  };

  uint8_t readbuff[READLENGTH];

  uint8_t testPhase;
  error_t signalError;
  uint8_t packet[2];
  uint8_t state=S_OFF;
  int8_t position;



  command error_t Init.init() {
    /*
     * same power pin as gyro regulator, this will bring up second power pin (first tied to
     * shimmer regulator).  it turns on in idle mode
     */
    // power, active low
    TOSH_MAKE_PROG_OUT_OUTPUT();   
    TOSH_SEL_PROG_OUT_IOFUNC();
    TOSH_CLR_PROG_OUT_PIN();    // on

    TOSH_uwait(7000); // 5 ms for mag
    state=S_IDLE;
    position=-20;
    return SUCCESS;
  }
  
  command void Magnetometer.getLastRead(uint8_t *data){
      memcpy(data,readbuff,READLENGTH);
  }

  void writeRegValue(uint8_t reg_addr, uint8_t val) {
    // pack the packet with address of reg target, then register value
    packet[0] = reg_addr;
    packet[1] = val;
    if(call Resource.immediateRequest()==SUCCESS)
      call I2CPacket.write(I2C_START | I2C_STOP, 0x1e, WRITELENGTH, packet);
    else
      call Resource.request();
  }
  
  void readValues(uint8_t readpos,bool close){
    if(!close){
      call I2CPacket.read(I2C_START , 0x1e, READONCE, readbuff);
    }else{
      call I2CPacket.read(I2C_START|I2C_STOP , 0x1e, READONCE, readbuff);
    }
  }
  
  void startread(uint8_t from){
    packet[0]=from;
    if(call Resource.immediateRequest()==SUCCESS)
      //call I2CPacket.write(I2C_START , 0x1e, 1, packet);
      if(READLENGTH!=READONCE)
          readValues(++position,FALSE);
        else
          readValues(++position,TRUE);
    else
      call Resource.request(); 
  }


  
  event void Resource.granted(){
    if(state==S_READ||state==S_READ_CONTMEAS)
      call I2CPacket.write(I2C_START, 0x1e, 1, packet);
    else
      call I2CPacket.write(I2C_START | I2C_STOP, 0x1e, WRITELENGTH, packet);
  }
  
  /*
   * 0.5, 1, 2, 5, 10 (default), 20, 50hz.  20 and 50 up power burn dramatically
   * bits 2-4 control this, values 0 - 6 map to values above, respectively
   * since the remainder of the register defaults to 0, we write the mask directly
   */
  command error_t Magnetometer.setOutputRate(uint8_t rate){
    uint8_t bitmask;  // default 10hz
    
    if(state<64)
      return EBUSY;
    if(state==S_SLEEP)
      state=S_SETRATE_SLEEP;
    else if(state==S_CONTMEAS)
      state=S_SETRATE_CONTMEAS;
    else
    state=S_SETRATE;
    
    switch(rate){
    case ZERO_5_HZ:
      bitmask = 0;
      break;
    case ONE_HZ:
      bitmask = 0x04;
      break;
    case TWO_HZ:
      bitmask = 0x08;
      break;
    case FIVE_HZ:
      bitmask = 0x0c;
      break;
    case TEN_HZ:
      bitmask = 0x10;
      break;
    case TWENTY_HZ:
      bitmask = 0x14;
      break;
    case FIFTY_HZ:
      bitmask = 0x18;
      break;
    default:
      return EINVAL;  // input value unknown, don't set anything
      break;
    }
    writeRegValue(0, bitmask);
    
    return SUCCESS;
  }

  // +-0.7, 1.0 (default), 1.5, 2.0, 3.2, 3.8, 4.5Ga
  command error_t Magnetometer.setGain(uint8_t gain){ 
    uint8_t  bitmask;  // default 1.0Ga
    
    if(state<64)
      return EBUSY;
    if(state==S_SLEEP)
      state=S_SETGAIN_SLEEP;
    else if(state==S_CONTMEAS)
      state=S_SETGAIN_CONTMEAS;
    else
      state=S_SETGAIN;
    
    switch(gain){
    case ZERO_7_GAUSS:
      bitmask = 0x00;
      break;
    case ONE_GAUSS:
      bitmask = 0x20;
      break;
    case ONE_5_GAUSS:
      bitmask = 0x40;
      break;
    case TWO_GAUSS:
      bitmask = 0x50;
      break;
    case THREE_2_GAUSS:
      bitmask = 0x80;
      break;
    case THREE_8_GAUSS:
      bitmask = 0x90;
      break;
    case FOUR_5_GAUSS:
      bitmask = 0xc0;
      break;
    default:
      return EINVAL;  // input value unknown, don't set anything
      break;
    }
    writeRegValue(1, bitmask);
    
    return SUCCESS;
  }

  command error_t Magnetometer.setIdle(){
    if(state==S_IDLE)
      return EALREADY;
    else if(state<64)
      return EBUSY;
    state=S_GOIDLE;
    
    writeRegValue(2, 0x02);
    return SUCCESS;
  }

  command error_t Magnetometer.goToSleep(){
    if(state==S_SLEEP)
      return EALREADY;
    else if(state<64)
      return EBUSY;
    state=S_GOSLEEP;
    
    writeRegValue(2, 0x03);
    return SUCCESS;
  }


  command error_t Magnetometer.runSingleConversion(){
    if(state<64)
      return EBUSY;
    state=S_RUN_SINGLE;
    writeRegValue(2, 0x01);
    return SUCCESS;
  }
    

  command error_t Magnetometer.runContinuousConversion(){
    if(state==S_CONTMEAS)
      return EALREADY;
    if(state<64)
      return EBUSY;
    state=S_RUN_CONT_MEAS;    
    writeRegValue(2, 0x00);
    return SUCCESS;
  }

  /*
   * returning the real value doesn't help; 
   * success is measured in the magreaddone event
   */
  command error_t Magnetometer.readData(){
    if(state<64)
      return EBUSY;
    if(state==S_CONTMEAS)
      state=S_READ_CONTMEAS;
    else if(state==S_SLEEP)
      state=S_READ_SLEEP;
    else
      state=S_READ;
    position=-1;
    //readValues(0);
    
    startread(3);
    return SUCCESS;
  }
  
  // call after readDone event
  command uint16_t Magnetometer.readHeading(int16_t * readVals){
    uint16_t heading;

    if(readVals[0] == 0){
      if(readVals[1] < 0)
    heading = 270;
      else
    heading = 90;
    }
    else if(readVals[2] < 0)
      heading = (uint16_t)(180.0 - atan2f((float)readVals[1], (float)-readVals[0]) * 57.3);

    else
      heading = (uint16_t)(180.0 - atan2f((float)readVals[1], (float)readVals[0]) * 57.3);
    return heading;
  }
  
  task void readDone(){
    error_t error;
    atomic {
      error=signalError;
    }
    if(state==S_READ_CONTMEAS)
      state=S_CONTMEAS;
    else if(state==S_READ_SLEEP)
      state=S_SLEEP;
    else
      state=S_IDLE;
    signal Magnetometer.readDone((int16_t*)readbuff, error);//TODO possible endianness bug: sensor is little endian
  }

  async event void I2CPacket.readDone(error_t success, uint16_t addr, uint8_t length, uint8_t* data) {

//     position+=READONCE;
//     if(position<READLENGTH-1)
//       readValues(position, FALSE);
//     else if(position==READLENGTH-1)
//       readValues(position,TRUE);
//       //readValues(position, FALSE);    
//     else
    {
      position=-20;
      call Resource.release();
      atomic{
        signalError=success;
      }
      post readDone();
     }
  }
  
  task void writeDone(){
    error_t error;
    atomic {
      error=signalError;
    }
    switch(state){
      case S_GOIDLE:{
        state=S_IDLE;
        signal Magnetometer.setIdleDone(error);
      }break;
      case S_GOSLEEP:{
        state=S_SLEEP;
        signal Magnetometer.goToSleepDone(error);
      }break;
      case S_RUN_CONT_MEAS:{
        state=S_CONTMEAS;
        signal Magnetometer.runContinuousConversionDone(error);
      }break;
      case S_RUN_SINGLE:{
        state=S_SLEEP;//it goes to sleep after measurement
        signal Magnetometer.runSingleConversionDone(error);
      }break;
      case S_SETGAIN:{
        state=S_IDLE;
        signal Magnetometer.setGainDone(error);
      }break;
      case S_SETGAIN_CONTMEAS:{
        state=S_CONTMEAS;
        signal Magnetometer.setGainDone(error);
      }break;
      case S_SETGAIN_SLEEP:{
        state=S_CONTMEAS;
        signal Magnetometer.setGainDone(error);
      }break;
      case S_SETRATE:{
        state=S_IDLE;
        signal Magnetometer.setOutputRateDone(error);
      }break;
      case S_SETRATE_CONTMEAS:{
        state=S_CONTMEAS;
        signal Magnetometer.setOutputRateDone(error);
      }break;
      case S_SETRATE_SLEEP:{
        state=S_SLEEP;
        signal Magnetometer.setOutputRateDone(error);
      }break;
    }
  }

  async event void I2CPacket.writeDone(error_t success, uint16_t addr, uint8_t length, uint8_t* data) {   
    if(position==-1){//we're reading
        if(READLENGTH!=READONCE)
          readValues(++position,FALSE);
        else
          readValues(++position,TRUE);
    }else{
      atomic{
        signalError=success;
      }
      call Resource.release();
      post writeDone();
    }
  }
}
  





