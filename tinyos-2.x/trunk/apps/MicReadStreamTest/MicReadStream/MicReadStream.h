/** Copyright (c) 2009, University of Szeged
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
* Author: Zoltan Kincses
*/

#ifndef MICREADSTREAM_H
#define MICREADSTREAM_H

enum{
	AM_DATAMSG = 1,
	AM_CTRLMSG = 2,
	AM_READYMSG=3,
	MIC_SAMPLES = 112,
	TIMER_PERIOD= 10240,
};

typedef nx_struct datamsg{
	nx_uint8_t micData[MIC_SAMPLES];
	nx_uint16_t sampleNum;
}datamsg_t;

typedef nx_struct readymsg{
	nx_uint32_t usActualPeriod;
	nx_uint16_t sampleNum;
	nx_uint16_t bufferDoneNum;
	nx_uint16_t sendErrorNum;
	nx_uint16_t sendDoneErrorNum;
	nx_uint16_t busyTrueNum;
}readymsg_t;


typedef nx_struct ctrlmsg{
	nx_uint8_t instr;
	nx_uint16_t micPeriod;
}ctrlmsg_t;

#endif
