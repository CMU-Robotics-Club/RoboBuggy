package com.roboclub.robobuggy.main;

import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.SwingConstants;

import com.roboclub.robobuggy.serial.SerialListener;
import com.roboclub.robobuggy.serial.SerialReader;

abstract class SerialPanel extends JPanel {
	private static final long serialVersionUID = 8010633614957618540L;
	
	private SerialReader port;
	
	public SerialPanel(String owner, int baud_rate, char[] header,
			int headerLen) {
		try {
			port = new SerialReader(owner, baud_rate, header, headerLen);
		} catch (Exception e) {
			//TODO add error handling for serial ports
		}
	}
	
	public void closePort() {
		port.close();
	}
	
	public SerialReader getPort() {
		return port;
	}
	public void addListener(SerialListener listener) {
		if (null != port) {
			port.addListener(listener);
		}
	}
}