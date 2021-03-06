#include "cmd.h"
#include "parameters.h"

#define STARTTIME 1000
#define STOPTIME 1000
#define ANSTIMEOUT 5000
#define DELAY_T 3000


module GSMDriverP{
	provides interface GsmControl;
	uses
	{
		interface GeneralIO as ON;
		interface Timer<TMilli> as TimerAnswer;
		interface Timer<TMilli> as TimerPower;
		interface Timer<TMilli> as TimerDelay;
		interface UartStream;
		interface StdControl;
		
	}
}
implementation{
	
	enum{
		OFFED=0,
		OFFING,
		ONING,
		ONED,
		SENDING
	};
	
	norace error_t err;
	norace uint8_t state=OFFED,len1,i; 
	char cmd_data[(TOSH_DATA_LENGTH*2)+16];
	char ans[105],cmd[20],receive[28];
	char temp[2],g_msg[TOSH_DATA_LENGTH];		
	char cmd_disconnect[12] ="AT+CIPCLOSE\r"; 
	char cmd_connect[130];
	char diagString[10];

	task void connectTask(){
		signal GsmControl.connectToGSMDone(err);
	}
	
	task void disconnectTask(){
		signal GsmControl.disconnectFromGSMDone(err);
	}
	
	task void sendTask(){
		signal GsmControl.sendToGSMDone(err);
	}
	task void stopTimer(){
		call TimerAnswer.stop();
	}
	
	task void receiveTask(){
		signal GsmControl.receivedData(receive);
	}

	task void offTask(){
		call StdControl.stop();
		call UartStream.disableReceiveInterrupt();
		call ON.set();
		call TimerPower.startOneShot(STOPTIME);
	}
		
	command error_t GsmControl.connectToGSM(){
		error_t error;
		if (state==ONING) {
			error=SUCCESS;
		}else if (state==ONED) {
			error=EALREADY;
		}else if (state==OFFING || state==SENDING) {
			error=EBUSY;
		}else{
			error=SUCCESS;
			state=ONING;
			call ON.set();
			call TimerPower.startOneShot(STARTTIME);			
		}		
		return error;
	}
	
	command error_t GsmControl.disconnectFromGSM(){
		error_t error;
		if (state==OFFING){
			error=SUCCESS;
		}else if (state==OFFED) {
			error=EALREADY;
		}else if (state==ONING){
			error=EBUSY;
		}else{		
			state=OFFING;
			error=SUCCESS;
			call TimerDelay.startOneShot(DELAY_T);
		}
		return error;
	}
	
	event void TimerPower.fired(){ 
		call ON.clr();
		if(state==ONING){
			call StdControl.start();
			call UartStream.enableReceiveInterrupt();
			call TimerDelay.startOneShot(CONNECT_TIMEOUT);
		}
		if(state==OFFING){
			state=OFFED;
			err=SUCCESS;
			post disconnectTask();
		}
	}
	
	event void TimerDelay.fired(){
		if(state==ONING){
			strcpy(cmd_connect, "AT+CIPSHUT;");
			strcat(cmd_connect, "+CIPMUX=0;");
			strcat(cmd_connect, "+CSTT=\"");
			strcat(cmd_connect, APN);
			strcat(cmd_connect, "\",\"");
			strcat(cmd_connect, USER);
			strcat(cmd_connect, "\",\"");
			strcat(cmd_connect, PWD);
			strcat(cmd_connect, "\";");
			strcat(cmd_connect, "+CIICR;");
			strcat(cmd_connect, "+CIFSR;");
			strcat(cmd_connect, "+CIPSTART=\"");
			strcat(cmd_connect, UDP_TCP);
			strcat(cmd_connect, "\",\"");
			strcat(cmd_connect, IP);
			strcat(cmd_connect, "\",\"");
			strcat(cmd_connect, SERVERPORT);
			strcat(cmd_connect, "\";+CIPSPRT=0");
			strcat(cmd_connect,"\r");
			call UartStream.send((uint8_t*)cmd_connect,strlen(cmd_connect));	
			call TimerAnswer.startOneShot(ANSTIMEOUT);
		}
		if(state==SENDING){
			call UartStream.send((uint8_t*)g_msg,len1);
		}
		if(state==OFFING){
			call UartStream.send((uint8_t*)cmd_disconnect,12);		
		}
	
	}
		
	event void TimerAnswer.fired(){ 
		err=FAIL;
		if (state==ONING){
			post connectTask();
		}else if (state==OFFING){
			post disconnectTask();
		}else{
			post sendTask();
		}
	}
	
	
	command error_t GsmControl.SendToGSM(char* msg){
		if (state==ONING || state==OFFING || state==OFFED)
			return FAIL;
		else{
			strcpy(g_msg,msg);
			len1=strlen(g_msg);
			intToString(temp,len1);
			strcpy(cmd,"AT+CIPSEND=");
			strcat(cmd,temp);
			strcat(cmd,"\r");
			if (call UartStream.send((uint8_t*)cmd,strlen(cmd))!=SUCCESS){
				return FAIL;
			}else{
				state=SENDING;
				call TimerDelay.startOneShot(DELAY_T);
				return SUCCESS;
			}
		}
	}

	async event void UartStream.receivedByte( uint8_t byte ){
		static uint8_t byteCounter=0,pos=0,cmp=0;
		
		ans[byteCounter++]=byte;
		if(byte==10){
			if(state==ONING){
				switch(pos++){
					case(0):	
						post stopTimer();
						byteCounter=0;						
						break;
					case(1):
						cmp=strncmp(ans,"SHUT OK",7);
						byteCounter=0;						
						break;
					case(2):
						byteCounter=0;						
						break;
					case(3):
						cmp=strncmp(ans,"OK",2);
						byteCounter=0;						
						break;
					case(4):
						byteCounter=0;						
						break;
					case(5):
						byteCounter=0;
						break;
					case(6):
						byteCounter=0;
						break;
					case(7):
						cmp=strncmp(ans,"OK",2);
						byteCounter=0;
						break;
					case(8):
						byteCounter=0;						
						break;
					case(9):
						cmp=strncmp(ans,"CONNECT OK",10);
						byteCounter=0;
						break;
				}
				if (cmp!=0){
					err=FAIL;
					pos=0;
					state=OFFING;
					post offTask();
				}else if (pos==10){
					err=SUCCESS;
					pos=0;
					state=ONED;
					post connectTask();
				}
			}else if(state==OFFING){
				switch(pos++){
					case(0):
						post stopTimer();
						byteCounter=0;
						break;
					case(1):
						cmp=strncmp(ans,"CLOSE OK",8);
						byteCounter=0;
						break;
				}
				if (cmp!=0){
					err=FAIL;
					pos=0;	
					state=ONED;
				}else{
					err=SUCCESS;
					post offTask();
				}
			}else if(state==SENDING){
				switch(pos++){
				case(0):	
					post stopTimer();
					byteCounter=0;
					break;
				case(1):
					post stopTimer();
					cmp=strncmp(ans,"SEND OK",7);
					byteCounter=0;
					break;
				}
				if (cmp!=0){
					err=FAIL;
					post sendTask();
					state=ONED;
				}else if (pos==1){
					err=SUCCESS;
					state=ONED;
					post sendTask();
				}
			} else {
				strcpy(receive,ans);
				byteCounter=0;
				post receiveTask();
			}
		}
	}
	
	async event void UartStream.sendDone( uint8_t* buf, uint16_t len, error_t error ){
		err=error;
		if(err!=SUCCESS){
			if (state==ONING){
				state=OFFED;
				post connectTask();
			}else if (state==OFFING){
				state=ONED;
				post disconnectTask();
			}else{
				state=ONED;
				post sendTask();
			}
		}
	}	
	
	async event void UartStream.receiveDone( uint8_t* buf, uint16_t len, error_t error ){}

}
