// $Id: DfrfGradTestC.nc,v 1.4 2010-08-11 09:20:28 andrasbiro Exp $

/*									tab:4
 * "Copyright (c) 2000-2003 The Regents of the University  of California.  
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author appear in all copies of this software.
 * 
 * IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO ANY PARTY FOR
 * DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
 * OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF
 * CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE UNIVERSITY OF CALIFORNIA SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS."
 *
 * Copyright (c) 2002-2003 Intel Corporation
 * All rights reserved.
 *
 * This file is distributed under the terms in the attached INTEL-LICENSE     
 * file. If you do not find these files, copies can be found by writing to
 * Intel Research Berkeley, 2150 Shattuck Avenue, Suite 1300, Berkeley, CA, 
 * 94704.  Attention:  Intel License Inquiry.
 */

/**
 * The TinyOS 2.x base station that forwards packets between the UART
 * and radio.It replaces the GenericBase of TinyOS 1.0 and the
 * TOSBase of TinyOS 1.1.
 *
 * <p>On the serial link, BaseStation sends and receives simple active
 * messages (not particular radio packets): on the radio link, it
 * sends radio active messages, whose format depends on the network
 * stack being used. BaseStation will copy its compiled-in group ID to
 * messages moving from the serial link to the radio, and will filter
 * out incoming radio messages that do not contain that group ID.</p>
 *
 * <p>BaseStation includes queues in both directions, with a guarantee
 * that once a message enters a queue, it will eventually leave on the
 * other interface. The queues allow the BaseStation to handle load
 * spikes.</p>
 *
 * <p>BaseStation acknowledges a message arriving over the serial link
 * only if that message was successfully enqueued for delivery to the
 * radio link.</p>
 *
 * <p>The LEDS are programmed to toggle as follows:</p>
 * <ul>
 * <li><b>RED Toggle:</b>: Message bridged from serial to radio</li>
 * <li><b>GREEN Toggle:</b> Message bridged from radio to serial</li>
 * <li><b>YELLOW/BLUE Toggle:</b> Dropped message due to queue overflow in either direction</li>
 * </ul>
 *
 * @author Phil Buonadonna
 * @author Gilman Tolle
 * @author David Gay
 * @author Philip Levis
 * @date August 10 2005
 */
#include "CounterPacket.h"

configuration DfrfGradTestC {
}
implementation {
  components MainC, DfrfGradTestP, LedsC, RandomC, ConvergecastC;
  components ActiveMessageC as Radio, SerialActiveMessageC as Serial;
  
  components GradientPolicyC as ConvergecastPolicyC;
  components new DfrfClientC(APPID_COUNTER, sizeof(counter_packet_t), sizeof(counter_packet_t), 15) as DfrfMainService;

  components new BroadcastClientC(APPID_COUNTER+1, sizeof(counter_packet_t)) as DfrfFieldService;

  DfrfGradTestP.FieldSend -> DfrfFieldService;
  DfrfGradTestP.FieldReceive -> DfrfFieldService;
  
  DfrfGradTestP.DfrfSend -> DfrfMainService;
  DfrfGradTestP.DfrfReceive -> DfrfMainService;
  DfrfMainService.DfrfPolicy -> ConvergecastPolicyC;
  
  MainC.Boot <- DfrfGradTestP;
  
  DfrfGradTestP.Init -> ConvergecastC;

  DfrfGradTestP.SerialControl -> Serial;
  DfrfGradTestP.RadioControl -> Radio;
  
  DfrfGradTestP.UartSend -> Serial;
  DfrfGradTestP.UartPacket -> Serial;
  DfrfGradTestP.UartAMPacket -> Serial;
  
  DfrfGradTestP.RadioPacket -> Radio;
  DfrfGradTestP.RadioAMPacket -> Radio;
  
  DfrfGradTestP.Convergecast -> ConvergecastC;
  DfrfGradTestP.Random -> RandomC;
  DfrfGradTestP.RandomSeedInit -> RandomC.SeedInit;
  
  DfrfGradTestP.Leds -> LedsC;
}
