/** Copyright (c) 2010, University of Szeged
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

module AccelAppP {

   uses {
   		interface Boot;
		interface LedHandler;
		//interface SplitControl as TimeSyncMsg;
		interface SplitControl as DiskCtrl;
		interface StdControl as MeterCtrl;
		interface Timer<TMilli> as LagTimer;
		interface SplitControl as Sampling;
		//interface StdControl as SyncMsgCtrl;
		interface Init as AutoZero;
   }
}

implementation {

	event void Boot.booted() {
		
		error_t error;
		
		call LedHandler.led1On();
		
		error = call DiskCtrl.start();
		
		if (error) {
			call LedHandler.error();
		}
	}

	event void DiskCtrl.startDone(error_t error){
		
		if (error) {
			call LedHandler.error();
			return;
		}
		
		error = call MeterCtrl.start();
		
		if (error) {
			call LedHandler.error();
		}
		else {
			call LagTimer.startOneShot(3000);
		}
		

	}
	
	event void LagTimer.fired() {
	  
		error_t error;
	  
		call AutoZero.init();	  // gyro autozero, hopes it is stationary now
	
		error = call Sampling.start();
		
		if (error) {
			call LedHandler.error();
		}
		
	}
	
	event void Sampling.startDone(error_t error) {
		
		if (error) {
			call LedHandler.error();
			return;
		}
		
		call LedHandler.led1Off();
	}

	event void DiskCtrl.stopDone(error_t error) {
		call LedHandler.error();
	}

	event void Sampling.stopDone(error_t error) {
		call LedHandler.error();
	}
}
