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
* Author:Andras Biro
*/
generic configuration StreamUploaderC(uint8_t am_id){
	provides interface StdControl;
}
implementation{
	components StreamUploaderP as App;
	components new AMSenderC(am_id) as AMSend;
	components new AMReceiverC(am_id) as AMReceive;
	components ActiveMessageC;
	components new TimerMilliC() as WaitTimer;
	components new TimerMilliC() as StorageWaitTimer;
	components LedsC, LocalTimeMilliC, TimeSyncMessageC;
	
	App.Leds->LedsC;
	App.Packet -> AMSend;
 	App.AMPacket -> AMSend;
  	App.AMSend -> AMSend;
  	App.SplitControl -> ActiveMessageC;
  	App.PacketAcknowledgements -> ActiveMessageC;
  	App.Receive -> AMReceive;
  	App.WaitTimer->WaitTimer;
  	App.StorageWaitTimer->StorageWaitTimer;
  	//App.PacketTimeStampMilli -> TimeSyncMessageC.PacketTimeStampMilli;
  	//App.TimeSyncPacketMilli -> TimeSyncMessageC.TimeSyncPacketMilli;
  	App.TimeSyncAMSendMilli -> TimeSyncMessageC.TimeSyncAMSendMilli[am_id+1];
  	App.LocalTime -> LocalTimeMilliC;
  	StdControl=App.StdControl;
}