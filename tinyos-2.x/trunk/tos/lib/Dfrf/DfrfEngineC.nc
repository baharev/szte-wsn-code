/*
 * Copyright (c) 2009, Vanderbilt University
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author appear in all copies of this software.
 * 
 * IN NO EVENT SHALL THE VANDERBILT UNIVERSITY BE LIABLE TO ANY PARTY FOR
 * DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
 * OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE VANDERBILT
 * UNIVERSITY HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE VANDERBILT UNIVERSITY SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND THE VANDERBILT UNIVERSITY HAS NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 *
 * Author: Janos Sallai
 */

configuration DfrfEngineC {
  provides {
    interface DfrfControl[uint8_t appId];
    interface DfrfSend<uint8_t> as DfrfSend[uint8_t appId];
    interface DfrfReceive<uint8_t> as DfrfReceive[uint8_t appId];
  }
  uses {
    interface DfrfPolicy[uint8_t appId];
  }
} implementation {

  components MainC, DfrfEngineP as Engine, ActiveMessageC, new TimerMilliC() as DfrfTimer, NoLedsC as LedsC;

  DfrfSend = Engine.DfrfSend;
  DfrfReceive = Engine.DfrfReceive;

  Engine.DfrfControl = DfrfControl;
  Engine.DfrfPolicy = DfrfPolicy;

  Engine.Timer -> DfrfTimer;
  Engine.Leds -> LedsC;

  Engine.AMSend -> ActiveMessageC.AMSend[AM_DFRF_MSG];
  Engine.Receive -> ActiveMessageC.Receive[AM_DFRF_MSG];
  Engine.Packet -> ActiveMessageC;
}
