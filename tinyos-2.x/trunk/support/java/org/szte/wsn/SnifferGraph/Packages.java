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
* */

package org.szte.wsn.SnifferGraph;
import java.awt.Color;
import java.util.ArrayList;

import javax.swing.BorderFactory;
import javax.swing.JCheckBox;
import javax.swing.JPanel;
import javax.swing.JTextField;

public class Packages {

	protected int startTime;
	protected int stopTime;
	protected int timeLenght;
	protected int firstPos;
	protected String[] labelNames;
	protected JTextField[] cells;
	protected JPanel motePanel;
	
	//getters and setters
	public int getFirstPos() {
		return firstPos;
	}

	public void setFirstPos(int firstPos) {
		this.firstPos = firstPos;
	}
	
	public int getStartTime() {
		return startTime;
	}
	
	public void setStartTime(int startTime) {
		this.startTime = startTime;
	}
	
	public int getStopTime() {
		return stopTime;
	}
	
	public void setStopTime(int stopTime) {
		this.stopTime = stopTime;
	}
	
	public int getTimeLenght() {
		return timeLenght;
	}
	
	public void setTimeLenght(int timeLenght) {
		this.timeLenght = timeLenght;
	}
	
	public String[] getLabelNames() {
		return labelNames;
	}
	
	public void setLabelNames(String[] labelNames) {
		this.labelNames = labelNames;
	}
	
	public JTextField[] getCells() {
		return cells;
	}
	
	public void setCells(JTextField[] cells) {
		this.cells = cells;
	}
	
	public JPanel getMotePanel() {
		return motePanel;
	}
	
	public void setMotePanel(JPanel motePanel) {
		this.motePanel = motePanel;
	}
	
	//other futures
	/**
	 * Restore a panel with the given properties mote draws
	 * 
	 * @return motPanel: The panel requested features drawn
	 *  */
	public JPanel getAPanel(ArrayList<JCheckBox> devices){
		this.firstPos = 130+this.startTime*100;
		motePanel = new JPanel();
		motePanel.setLayout(null);
		motePanel.setBounds(firstPos, 270, this.timeLenght*100, 2*70-50);		//bejelöld dobozok * dobozméret - egy fél dobozméret
		motePanel.setBackground(Color.orange);
		motePanel.setBorder(BorderFactory.createEtchedBorder());
		int k = 0, j = 0;
		for(int i = 0; i<devices.size(); i++){
			if(devices.get(i).isSelected()){
				cells[j] = new JTextField(this.labelNames[i]);
				cells[j].setBackground(null);
				cells[j].setBorder(BorderFactory.createEtchedBorder());
				cells[j].setBounds(0, k, this.timeLenght*100, 20);
				motePanel.add(cells[j++]);
				k+=70;
			}
		}
		motePanel.setVisible(true);
		return motePanel;
	}
	
	public JPanel getALilPanel(ArrayList<JCheckBox> devices, ArrayList<String> colors){
		motePanel = new JPanel();
		
		return motePanel; 
	}
	
	//konstrukor
	public Packages(int startTime, int stopTime, String[] labelNames) {
		super();
		this.startTime = startTime;
		this.stopTime = stopTime;
		this.timeLenght = stopTime - startTime;
		this.labelNames = new String[labelNames.length];
		
		for(int i = 0; i < labelNames.length; i++){
			this.labelNames[i] = labelNames[i];
		}
	}	
}