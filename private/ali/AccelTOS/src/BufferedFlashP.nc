/**
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
* Author: Miklos Maroti
*/

#include "Assert.h"
#include "TimeSyncInfo.h"

module BufferedFlashP
{
	provides
	{
		interface BufferedFlash;
	}

	uses
	{
		interface SimpleFile;
		interface LedHandler;
		interface DiagMsg;
	}
}

implementation
{
	enum
	{
		BUFFER_SIZE = 2,
		 // FIXME It should not know these implementation details
		// ((block size) - (header length) - (sizeof time sync msg) ) / (sample size) 
		// (512 - 10 - 14)/22 = 22
		// 22*22 + 14 = 498; dataloss if not exactly set!!!
		MAX_DATA_LENGTH = 498
	};
	
	typedef struct {
		uint16_t length;
		uint8_t data[MAX_DATA_LENGTH];
	} buffer_t;

	buffer_t messages[BUFFER_SIZE];

	uint8_t current;	// the currently recorded message buffer
	uint16_t position;	// the write position in the current buffer
	uint8_t pending;	// the number of full messages
	bool sending; // FIXME Guards what? messages?
/*	
	void dumpUint32(char* msg, uint32_t i) {
		if( call DiagMsg.record() ) {
			call DiagMsg.str(msg);
			call DiagMsg.uint32(i);
			call DiagMsg.send();
		}		
	}
	
	void dumpPending(uint16_t line) {
		if( call DiagMsg.record() ) {
			call DiagMsg.uint16(line);
			call DiagMsg.uint8(pending);
			call DiagMsg.send();
		}		
	}
*/	

	void dumpEbusy(uint16_t line) {
		if( call DiagMsg.record() ) {
			call DiagMsg.uint16(TOS_NODE_ID);
			call DiagMsg.str("EBUSY");
			call DiagMsg.uint16(line);
			call DiagMsg.send();
		}		
	}

	task void sendMessage()
	{

		if (pending > BUFFER_SIZE) {
			ASSERT(FALSE);
			return;
		}
		
		if( ! sending && pending > 0 )
		{
			int8_t first = current - pending;
			if( first < 0 )
				first += BUFFER_SIZE;
				
			if (first < 0 || first >= BUFFER_SIZE) {
				ASSERT(FALSE);
				return;	
			}

			if( call SimpleFile.append(messages[first].data, messages[first].length) == SUCCESS ) {
				sending = TRUE;
			}
			else {
				dumpEbusy(__LINE__);
				post sendMessage();
			}
		}
		
		//dumpPending(__LINE__);
	}

	command error_t BufferedFlash.send(void *data, uint8_t length)
	{
		//dumpPending(__LINE__);
		
		if (pending > BUFFER_SIZE) {
			ASSERT(FALSE);
			return FAIL;
		}
		
		if (pending == BUFFER_SIZE) {
			dumpEbusy(__LINE__);
			return EBUSY;
		}
			
		if( position + length > MAX_DATA_LENGTH )
		{
			call BufferedFlash.flush(); // Just posts a task and increments current

			if(pending == BUFFER_SIZE) {
				dumpEbusy(__LINE__); // FIXME We get here if flush is not called in a timely manner
				return EBUSY;
			}
		}
		
		if (current >= BUFFER_SIZE) {
			ASSERT(FALSE);
			return FAIL;	
		}
		
		if (position==0) {
			// TODO ask SyncMsgReceiver for timesync_info
			// memcpy(messages[current].data, &timesync_info, sizeof(timesync_info));
			memset(messages[current].data, 0, sizeof(timesync_info_t));
			position = sizeof(timesync_info_t);
		}
		
		// FIXME Should properly check if the size of buffer is enough		
		ASSERT(position+length<=MAX_DATA_LENGTH);
		
		memcpy(messages[current].data + position, data, length);
		position += length;

		if( position == MAX_DATA_LENGTH ) { // FIXME This check is not enough
			call BufferedFlash.flush();
		}
		else if (position > MAX_DATA_LENGTH) {
			ASSERT(FALSE);
			return FAIL;
		}

		return SUCCESS;
	}

	event void SimpleFile.appendDone(error_t error){
		
		sending = FALSE;
	
		if( error == SUCCESS ) {
			--pending;
			ASSERT(pending<BUFFER_SIZE);
		}
		else {
			dumpEbusy(__LINE__); // FIXME How can SD writeBlock fail?
			post sendMessage();
			ASSERT(pending<=BUFFER_SIZE);
		}
		
		//dumpPending(__LINE__);
	}

	command void BufferedFlash.flush()
	{
		
		ASSERT(pending<BUFFER_SIZE);
		
		if( position > 0 )
		{

			if (current >= BUFFER_SIZE || position>MAX_DATA_LENGTH) {
				ASSERT(FALSE);
				return;	
			}
			// store the length
			messages[current].length = position;

			position = 0;
			if( ++current >= BUFFER_SIZE )
				current = 0;

			++pending;
			post sendMessage();
		}
		
		//dumpPending(__LINE__);
	}

	event void SimpleFile.formatDone(error_t error){
		// TODO Auto-generated method stub
	}

	event void SimpleFile.seekDone(error_t error){
		// TODO Auto-generated method stub
	}

	event void SimpleFile.readDone(error_t error, uint16_t length){
		// TODO Auto-generated method stub
	}
}
