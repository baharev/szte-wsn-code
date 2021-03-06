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
* Author: Csepe Zoltan
*/

import static java.lang.System.out;
import net.tinyos.packet.*;
import net.tinyos.message.*;
import net.tinyos.util.PrintStreamMessenger;
import java.io.*;

class SerialReceive implements MessageListener{
	
	private MoteIF moteIF;
	static PrintWriter fop;
	double last=-1;
	public SerialReceive(MoteIF moteIF){
		this.moteIF=moteIF;
		this.moteIF.registerListener(new SerialMsg(),this);
	}
	
	public void messageReceived(int dest_addr,Message msg){
		int tempmert;
		double tempkorr;
		double time;
		int node;
		if (msg instanceof SerialMsg) {
			SerialMsg serialData = (SerialMsg)msg;
			SerialPacket serPkt = msg.getSerialPacket();
			tempmert = serialData.get_temperature();
			tempkorr= (-39.6)+0.01*tempmert;
			time=serialData.get_time();
			node=serialData.get_nodeID();
			try{
		fop = new PrintWriter(new FileWriter("temp"+node+".txt",true));
		}catch(IOException ex){}
			if (last==time){System.exit(1);}
			else {last=time;}
			out.println("Temp_measured: "+tempmert+"Temperature: " + tempkorr +" Time "+time+" Node: "+node);
			fop.println("Temp_measured: "+tempmert+"Temperature: " + tempkorr +" Time "+time+" Node: "+node);
			fop.close();
		}
	}
	
	public void send(String counter, String counter2)  
	{
		SerialMsg msg=new SerialMsg();
		try{
			msg.set_counter(Integer.parseInt(counter));
			msg.set_counter2(Integer.parseInt(counter2));
			moteIF.send(MoteIF.TOS_BCAST_ADDR,msg);
			out.println("Message sent to the mote ");
		}catch(IOException e)
		{
			out.println("Cannot send message to mote ");
		}
	}

	public static void main(String[] args)throws IOException  
	{
		PhoenixSource phoenix = null;
		MoteIF mif = null;
		String data=null;
		String data2=null;
		int seged=-1;
				
		if( args.length == 1 ){
			phoenix = BuildSource.makePhoenix(PrintStreamMessenger.err);
			data=args[0];
		} else if ( args.length == 4 && args[0].equals("-comm") ) {
			phoenix = BuildSource.makePhoenix(args[1], PrintStreamMessenger.err);
			data=args[2];
			data2=args[3];
		} else {
			System.err.println("usage: java TestSerial [-comm <source>] [data to send] [ID to send]");
			System.exit(1);
		}
		mif = new MoteIF(phoenix);
		SerialReceive app= new SerialReceive(mif);
		app.send(data,data2);
		seged=Integer.parseInt(data);
		if(seged!=2) {
			System.exit(0);
		}
		else{out.println("letoltes");}
		
	}
}

	

	
	
	
