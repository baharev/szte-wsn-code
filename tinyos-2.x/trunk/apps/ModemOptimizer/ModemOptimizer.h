/*
* Copyright (c) 2011, University of Szeged
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

#ifndef _MODEM_OPTIMIZER_H_
#define _MODEM_OPTIMIZER_H_

#ifndef MOPT_RUNTIME_MSEC
#define MOPT_RUNTIME_MSEC 1000
#endif

#if MOPT_PACKET_SIZE < 4
#error MOPT_PACKET_SIZE must be greater or equal than 4!
#endif

#define DIAGMSG_S(STR)	\
	atomic { if( call DiagMsg.record() ) { \
			call DiagMsg.str(STR); \
			call DiagMsg.send(); \
		}}
	
#define DIAGMSG_U32(STR,VAR)	\
	atomic { if( call DiagMsg.record() ) { \
			call DiagMsg.str(STR); \
			call DiagMsg.uint32(VAR); \
			call DiagMsg.send(); \
		}}

typedef struct mopt_t {

	uint16_t srequest;
	uint16_t sbusy;
	uint16_t saccept;
	uint16_t serror;
	uint16_t ssuccess;
	
	uint16_t rsync;
	uint16_t rerror;
	uint16_t rcrc;
	uint16_t rsuccess;
	
} moptimizer_t;

#endif
