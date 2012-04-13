#include <Timer.h>
#include "GreenHouse.h"

module GreenHouseP{
	
	uses{
		
		interface Boot;
		interface Leds;
		interface StdControl as CtpControl;
		interface RootControl;
		interface SplitControl as RadioControl;
		interface SplitControl as SerialControl;
		
		interface Timer<TMilli>;
		interface Read<uint16_t>;
	
		interface CollectionPacket;
		interface CtpInfo;
		interface CtpCongestion;
		
		interface Send as CtpSend;
		interface Receive as CtpReceive;
		
		interface AMSend as SerialSend;
	
		interface DiagMsg;
	}
}
implementation{
	/*
	enum {
		SEND_INTERVAL = 8192
	};
	*/
	message_t CTPpacket;	//�zenet k�ld�s�hez r�di�n.
	message_t UARTpacket;	//�zenet k�ld�s�hez soros porton.
	//message_t buffer;		//�zenet k�ld�s�hez soros porton.

	uint16_t seqno;
	uint8_t readings = 0;	//from 0 to NREADINGS
	//uint8_t serialmsgs = 0;	//from 0 to NSERIALMSGS
	
	//GH_Msg* ghmsg;
	
	bool CTPsendBusy = FALSE;
	bool UARTsendBusy = FALSE;

	task void uartSendTask();
	//task void uartSendTask2();

	//Piros LED = HIBA
	void errorBlink()	{ call Leds.led0Toggle(); }
	//Z�ld LED = M�R�S
	void radioreadBlink()	{ call Leds.led1Toggle(); }
	//K�k LED = SOROS PORTRA K�LD�S HIBA
	void serialErrorOn(){ call Leds.led2On(); }
	void serialErrorOff(){ call Leds.led2Off(); }
	
	event void Boot.booted(){
		call SerialControl.start();
		call RadioControl.start();
		call CtpControl.start();

		if (TOS_NODE_ID /*% 500*/ == 0) {
			call RootControl.setRoot();
		}
		
		seqno = 0;
		
		call Timer.startPeriodic(DEFAULT_INTERVAL);
	}
	
	event void RadioControl.startDone(error_t err) {
	
		if( err != SUCCESS );
			//errorBlink();
	}
	event void SerialControl.startDone(error_t err) {
	
		if( err != SUCCESS );
			//errorBlink();
	}
	
	event void RadioControl.stopDone(error_t err) {}
	event void SerialControl.stopDone(error_t err) {}
	
	event void Timer.fired() {
		
		if( readings < NREADINGS )	//�trakhat� lenne a Read.readDone()-ba.
									//Szint�n ott else �gon bufferel�s.
		{
			/*readings ellen�rz�se!!!!!!!!*/
			
			
			call Read.read();
			//readings++;	//Read.readDone() n�veli		?!
		
			radioreadBlink();
			/*
			if( call DiagMsg.record() )
			{
				call DiagMsg.str("Timer");
				call DiagMsg.uint8(readings);
				call DiagMsg.uint16(seqno);
				call DiagMsg.uint8(CTPsendBusy);
				call DiagMsg.send();
			}
			*/
		}
	}
	
	void sendMessage(void);
	
	event void Read.readDone(error_t result, uint16_t val ){
		GH_Msg* ghmsg = (GH_Msg*)call CtpSend.getPayload(&CTPpacket, sizeof(GH_Msg));
		
		if(result == SUCCESS){
			ghmsg->data[readings++] = /*-3960+*/(int32_t)val;
		}
		
		if(readings == NREADINGS)
		{
			sendMessage();
		}
	} 
	
	void sendMessage() {
		am_addr_t parent;
		uint16_t metric;
		
		GH_Msg* ghmsg = (GH_Msg*)call CtpSend.getPayload(&CTPpacket, sizeof(GH_Msg));
		
		if( call CtpInfo.getParent(&parent) == FAIL )
		{
			if( call DiagMsg.record() )
			{
				call DiagMsg.str("getParent failure...");
				call DiagMsg.send();
			}
		}
		if( call CtpInfo.getEtx(&metric) == FAIL )
		{
			if( call DiagMsg.record() )
			{
				call DiagMsg.str("getEtx failure...");
				call DiagMsg.send();
			}
		}

		ghmsg->source = TOS_NODE_ID;
		ghmsg->parent = parent;
		ghmsg->metric = metric;
		ghmsg->seqno = seqno;
		//ghmsg->hopcount = 0;
		
		if(!CTPsendBusy)
		{
			if ( call CtpSend.send(&CTPpacket, sizeof(GH_Msg))== SUCCESS)
			{
				/*
				if( call DiagMsg.record() )
				{
					call DiagMsg.str("CtpSend");
					call DiagMsg.uint16(ghmsg -> source);
					call DiagMsg.uint16(ghmsg -> seqno);
					call DiagMsg.uint16(ghmsg -> parent);
					call DiagMsg.uint16(ghmsg -> metric);
					call DiagMsg.send();
				}
				*/
				CTPsendBusy = TRUE;
				seqno++;
			}
			else
			{
				if( call DiagMsg.record() )
				{
					call DiagMsg.str("Packet was not accepted to send...");
					call DiagMsg.send();
				}
				//errorBlink();
			}
		}
	}
	
	event void CtpSend.sendDone(message_t* m, error_t err) {
		if (err != SUCCESS)
		{
			if( call DiagMsg.record() )
			{
				call DiagMsg.str("Failure sending packet...");
				call DiagMsg.send();
			}
			//errorBlink();
		}
		
		CTPsendBusy = FALSE;
		readings = 0;
		
	}
	
	message_t* /*ONE */receive(message_t* /*ONE */msg, void* payload, uint8_t len);
		
	event message_t* CtpReceive.receive(message_t* msg, void* payload, uint8_t len) {
		
		if( call RootControl.isRoot() )
		{
			//Send to serial port
			/*
			 * SerialSend -> AMSenderC konfigur�ci� eset�n
			 *********************************			
			memcpy(	call SerialSend.getPayload(&CTPpacket, sizeof(GH_Msg)), &ghmsg, sizeof(GH_Msg)	);
			if (call SerialSend.send(AM_BROADCAST_ADDR, &CTPpacket, sizeof(GH_Msg) ) == SUCCESS)
				UARTsendBusy = TRUE;
			 */
			receive(msg, payload, len);
		}
		/*
		GH_Msg* ghmsg = (GH_Msg*)payload;
		if( call DiagMsg.record() )
		{
			call DiagMsg.str("CtpReceive");
			call DiagMsg.uint16( ghmsg -> source);
			call DiagMsg.uint16( ghmsg -> seqno);
			call DiagMsg.uint16( ghmsg -> parent);
			call DiagMsg.uint16( ghmsg -> metric);
			call DiagMsg.uint16s( ghmsg -> data, 2 );
			call DiagMsg.send();
		}
		*/
		return msg;
	}
	
	message_t* receive(message_t *msg, void *payload, uint8_t len) {

		GH_Msg* ghmsg;
		/*
		ghmsg = (GH_Msg*)payload;
		
		if( call DiagMsg.record() )
		{
			call DiagMsg.str("CtpReceive");
			call DiagMsg.uint16( ghmsg -> source);
			call DiagMsg.uint16( ghmsg -> seqno);
			call DiagMsg.uint16( ghmsg -> parent);
			call DiagMsg.uint16( ghmsg -> metric);
			call DiagMsg.uint16s( (uint16_t*)(ghmsg -> data), 2 );
			call DiagMsg.send();
		}
		*/
		
		//atomic /*NEM KELL, mert taskok szinkron futnak*/
		//ha nincs async kulcssz� �S task kontextusban dolgozunk => nem kell atomic
		//{
			/*
			UART_Msg* uartmsg = (UART_Msg*)call SerialSend.getPayload(&UARTpacket, sizeof(UART_Msg));
			uartmsg -> messages[serialmsgs++] = 
			*/
			if (!UARTsendBusy)
			{
			/*
				if( call DiagMsg.record() )
				{
					call DiagMsg.str("Not busy.");
					call DiagMsg.send();
				}
			*/
				//�zenet M�SOL�SA
				//memcpy(&buffer, (GH_Msg*)payload, sizeof(GH_Msg));
				ghmsg = (GH_Msg*)call SerialSend.getPayload(&UARTpacket, sizeof(GH_Msg));
				*ghmsg = *((GH_Msg*)payload);
				
				if( call DiagMsg.record() )
				{
					call DiagMsg.str("CtpReceive2");
					call DiagMsg.uint16( ghmsg -> source);
					call DiagMsg.uint16( ghmsg -> seqno);
					call DiagMsg.uint16( ghmsg -> parent);
					call DiagMsg.uint16( ghmsg -> metric);
					call DiagMsg.uint16s( (uint16_t*)(ghmsg -> data), 2 );
					call DiagMsg.send();
				}
				
				post uartSendTask();
				UARTsendBusy = TRUE;	//El�z� sor el�??
			}
			else
			{
				if( call DiagMsg.record() )
				{
					call DiagMsg.str("Busy");
					call DiagMsg.send();
				}
			}
		//}
		return msg;
	}
  
	task void uartSendTask() {
		
		//if (call SerialSend.send(AM_BROADCAST_ADDR, &buffer, sizeof(GH_Msg)) == SUCCESS)
		if (call SerialSend.send(AM_BROADCAST_ADDR, &UARTpacket, sizeof(GH_Msg)) == SUCCESS)
		{
			serialErrorOff();
		}
		else
		{
			if( call DiagMsg.record() )
			{
				call DiagMsg.str("Task error.");
				call DiagMsg.send();
			}
			serialErrorOn();
			
			post uartSendTask();
			//post uartSendTask2();
		}
	}
	/*
	task void uartSendTask2() {
		
		if (call SerialSend.send(AM_BROADCAST_ADDR, &buffer, sizeof(GH_Msg)) == SUCCESS)
		{
		}
		else
		{
			if( call DiagMsg.record() )
			{
				call DiagMsg.str("Task error.");
				call DiagMsg.send();
			}
			serialsendBlink();
			
			post uartSendTask2();
		}
	}
	*/
	event void SerialSend.sendDone(message_t *msg, error_t error) {
		
		GH_Msg* ghmsg = (GH_Msg*)call SerialSend.getPayload(msg, sizeof(GH_Msg));
		
		if (error != SUCCESS)
		{
			if( call DiagMsg.record() )
			{
				call DiagMsg.str("SerialSend.sendDone error");
				call DiagMsg.send();
			}
			serialErrorOn();
		}
		else
		{
			serialErrorOff();
			
			if( call DiagMsg.record() )
			{
				call DiagMsg.str("SerialSend.sendDone");
				call DiagMsg.uint16( ghmsg -> source);
				call DiagMsg.uint16( ghmsg -> seqno);
				call DiagMsg.uint16( ghmsg -> parent);
				call DiagMsg.uint16( ghmsg -> metric);
				call DiagMsg.uint16s( (uint16_t*)(ghmsg -> data), 4 );
				call DiagMsg.send();
			}
		}
		
		UARTsendBusy = FALSE;
	}
	
}