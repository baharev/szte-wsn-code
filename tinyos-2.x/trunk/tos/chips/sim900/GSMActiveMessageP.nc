#include "cmd.h"

module GSMActiveMessageP{
	provides interface SplitControl;
	provides interface AMSend;
	provides interface Receive; 
	
	uses interface GsmControl;
	
}
implementation{
	
	message_t message,recmsg, *receiveptr=&recmsg;
	bool busy=FALSE;
	uint8_t reclen,i;
	
	
	command error_t SplitControl.start(){
		error_t err=call GsmControl.connectToGSM();
		return err;
	}

	event void GsmControl.connectToGSMDone(error_t err){
		signal SplitControl.startDone(err);
	}
		
	command error_t SplitControl.stop(){
		error_t err=call GsmControl.disconnectFromGSM();
		return err;
	}
	
	event void GsmControl.disconnectFromGSMDone(error_t err){
		signal SplitControl.stopDone(err);
	}
	
	command error_t AMSend.send(am_addr_t addr, message_t *msg, uint8_t len){
		error_t err;
		
		if(!busy){
			err=call GsmControl.SendToGSM(conToString1((uint8_t*)msg->data,TOS_NODE_ID,len,1,2,3,4));
			if(err==SUCCESS){
				busy=TRUE;
			}else{
				err=FAIL;
			}			
		}else{
			err=FAIL;	
		}
		return err;
	}
	
	event void GsmControl.sendToGSMDone(error_t err){
		busy=FALSE;
		signal AMSend.sendDone(&message, err);
	}

	command error_t AMSend.cancel(message_t *msg){
		return SUCCESS;
	}

	command void * AMSend.getPayload(message_t *msg, uint8_t len){
		return ((void*)msg)+sizeof(message_header_t);
	}

	command uint8_t AMSend.maxPayloadLength(){

		return 0;
	}

	event void GsmControl.receivedData(char* receiveMsg){
		reclen=strlen(receiveMsg);
		
		for (i = 0; i < reclen-1; i++) {
			receiveptr->data[i]=*receiveMsg++;
		}
				
		signal Receive.receive(receiveptr, ((void*)receiveptr)+sizeof(message_header_t), reclen);
	
	}
	
}