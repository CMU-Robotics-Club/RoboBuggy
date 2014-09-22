package com.roboclub.robobuggy.main;

import java.awt.Color;

import javax.swing.BorderFactory;
import javax.swing.JPanel;

import com.roboclub.robobuggy.serial.SerialListener;
import com.roboclub.robobuggy.serial.SerialReader;

abstract class SerialPanel extends JPanel {
	private static final long serialVersionUID = 8010633614957618540L;
	
	private SerialReader port;
	
	public SerialPanel(String owner, int baud_rate, char[] header,
			int headerLen) {
		this.setBorder(BorderFactory.createLineBorder(Color.black));
		try {
			port = new SerialReader(owner, baud_rate, header, headerLen);
		} catch (Exception e) {
			//TODO add error handling for serial ports
		}
	}
	
	public void closePort() {
		port.close();
	}
	
	protected void addListener(SerialListener listener) {
		if (null != port) {
			port.addListener(listener);
		}
	}
	
	protected boolean isConnected() {
		return (this.port != null && this.port.isConnected());
	}
}