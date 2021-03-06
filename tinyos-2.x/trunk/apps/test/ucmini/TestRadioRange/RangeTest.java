/*
 * Copyright (c) 2010, University of Szeged
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the copyright holder nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Andras Biro
 */

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;

import net.tinyos.message.Message;
import net.tinyos.message.MessageListener;
import net.tinyos.message.MoteIF;
import net.tinyos.packet.BuildSource;
import net.tinyos.packet.PhoenixSource;
import net.tinyos.util.PrintStreamMessenger;

public class RangeTest implements MessageListener{

  private MoteIF moteIF;
  static long counter=0;
  
  public RangeTest(MoteIF moteIF) {
    this.moteIF = moteIF;
    moteIF.registerListener(new DataMsg(), this);
  }
  
  public boolean waitForCommands(){
    BufferedReader in = new BufferedReader(new InputStreamReader(System.in));
    String s = null;
    try {
		while ((s = in.readLine()) != null){
		  String splitted[]=s.split(" ");
		  int dist=Integer.parseInt(splitted[0]);
			if(dist==9999)
				counter=0;
		  CommandMsg msg = new CommandMsg();
		  msg.set_distance(dist);
		  if(splitted.length>1){
			  int i=0;
			  while(i<CommandMsg.numElements_etc()&&i<splitted[1].length()){
				  msg.setElement_etc(i,(short) splitted[1].charAt(i));
				  i++;
			  }
		  }
		  moteIF.send(0,msg);
		  
		}
	} catch (NumberFormatException e) {
		if(!s.equals("")&&s!=null)
			System.err.println("Unparsable line");
	} catch (IOException e) {
		System.err.println("Communication error");
		return true;
	}
	if(s.equals("")||s==null)
		return true;
	else
		return false;
  }
  
  @Override
  public void messageReceived(int to, Message m) {
  	if(!(m instanceof DataMsg))
  		return;
  	DataMsg msg=(DataMsg)m;
  	String etc="";
  	for(short chr:msg.get_etc()){
  		etc+=String.valueOf((char)chr);
  	}
  	System.out.println("#"+ counter++ + ":" + etc+" dist="+msg.get_distance()+" mrssi="+msg.get_master_rssi()+" mlqi="+msg.get_master_lqi()+
  					   " srssi="+msg.get_slave_rssi()+" slqi="+msg.get_slave_lqi());
  }
  
  public static void main(String[] args) throws Exception {
    String source = null;
    if (args.length == 2) {
      if (!args[0].equals("-comm")) {
	System.exit(1);
      }
      source = args[1];
    }
    else if (args.length != 0) {
      System.err.println("Usage: RangeTest [-comm serial]");
      System.exit(1);
    }
    
    PhoenixSource phoenix;
    
    if (source == null) {
      phoenix = BuildSource.makePhoenix(PrintStreamMessenger.err);
    }
    else {
      phoenix = BuildSource.makePhoenix(source, PrintStreamMessenger.err);
    }

    MoteIF mif = new MoteIF(phoenix);
    RangeTest serial = new RangeTest(mif);
    while(!serial.waitForCommands());
    phoenix.shutdown();
  }

}
