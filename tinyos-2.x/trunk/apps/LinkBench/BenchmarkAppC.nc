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
* Author: Krisztian Veress
*         veresskrisztian@gmail.com
*/

#include "Messages.h"

configuration BenchmarkAppC {}

implementation {

  components MainC;
  Comm.Boot -> MainC.Boot;
  
  components BenchmarkCoreC as Core;
  components BenchmarkAppP as Comm;
  Comm.BenchmarkCore -> Core;
  Comm.CoreControl -> Core;
  Comm.CoreInit -> Core;

  components ActiveMessageC as Radio; 
  Comm.RadioControl -> Radio;
  Comm.Packet -> Radio;
  Comm.Receive -> Radio.Receive;

  components new DirectAMSenderC(AM_SYNCMSG_T)        as TxSync;
  components new DirectAMSenderC(AM_DATAMSG_T)        as TxData;
  Comm.TxSync -> TxSync;
  Comm.TxData -> TxData;
  
  components LedsC;
  Comm.Leds -> LedsC;
      
 
#ifdef TOSSIM

  components SerialActiveMessageC as Serial;
  components BaseStationP;
  
  Comm.TBSInit -> BaseStationP;
  Comm.RxCtrl -> BaseStationP.NullReceive1;
  Comm.RxSetup -> BaseStationP.NullReceive2;
  
  BaseStationP.Leds -> LedsC;
  
  BaseStationP.RadioControl -> Radio;
  BaseStationP.SerialControl -> Serial;
  
  BaseStationP.UartSend -> Serial;
  BaseStationP.UartReceive -> Serial.Receive;
  BaseStationP.UartPacket -> Serial;
  BaseStationP.UartAMPacket -> Serial;
  
  BaseStationP.RadioSend -> Radio;
  BaseStationP.RadioReceive -> Radio.Receive;
  BaseStationP.RadioSnoop -> Radio.Snoop;
  BaseStationP.RadioPacket -> Radio;
  BaseStationP.RadioAMPacket -> Radio;

#endif

}
