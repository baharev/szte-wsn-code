/*
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
 * Author:Miklos Toth
 */
package org.szte.wsn.dataprocess;

import java.io.IOException;
import org.szte.wsn.dataprocess.string.StringInterfaceFactory;
import org.szte.wsn.dataprocess.string.StringPacket;

public class Transfer extends Thread  {
	private PacketParser[] packetParsers;
	private BinaryInterface binary;
	private StringInterface string;
	boolean toString;

	public Transfer(String binaryType, String binaryPath, String stringType, String stringPath, String structPath, boolean toString){
		packetParsers=new PacketParserFactory(structPath).getParsers();
		binary=BinaryInterfaceFactory.getBinaryInterface(binaryType, binaryPath);	
		string=StringInterfaceFactory.getStringInterface(stringType, stringPath, packetParsers);
		this.toString=toString;
		if((binary==null)||(string==null))
			Usage.usageThanExit();
	}

	public Transfer(PacketParser[] packetParsers, BinaryInterface binary, StringInterface string, boolean toString){
		this.packetParsers=packetParsers;
		this.binary=binary;
		this.string=string;
		this.toString=toString;
	}

	@Override
	public void run(){
		if(toString){
			byte data[]=binary.readPacket();
			while(data!=null){
				for (PacketParser pp:packetParsers){
					if(pp.parse(data)!=null){
						string.writePacket(new StringPacket(pp.getName(),pp.getFields(),pp.parse(data)));		
					}

				}
				data=binary.readPacket();
			}	
		}
		else{			//other direction
			StringPacket sp=string.readPacket();
			while(sp!=null){
				PacketParser pp=PacketParserFactory.getParser(sp.getName(), packetParsers);
				if(pp.construct(sp.getData())!=null){
					try {
						binary.writePacket(pp.construct(sp.getData()));
					} catch (IOException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
						Usage.usageThanExit();
					}		
				}
				sp=string.readPacket();
			}
			
		}	

	}


	public static void main(String[] args) {			//handle exception		
		if(args.length<6)
			Usage.usageThanExit();
		PacketParser[] parsers=new PacketParserFactory(args[4]).getParsers();			
		BinaryInterface bin=BinaryInterfaceFactory.getBinaryInterface(args[0], args[1]);	
		StringInterface str=StringInterfaceFactory.getStringInterface(args[2], args[3],parsers);
		boolean toStr=false;
		if (args[5].equals("toString"))
			toStr=true;		

		Transfer fp=new Transfer(parsers,bin,str,toStr);
		fp.start();
	}

}