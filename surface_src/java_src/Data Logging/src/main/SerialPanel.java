package main;

import javax.swing.JPanel;

import serial.SerialListener;
import serial.SerialReader;

abstract class SerialPanel extends JPanel {
	private static final long serialVersionUID = 8010633614957618540L;
	
	private SerialReader port;
	private SerialListener listener;
	
	public SerialPanel(String owner, int baud_rate, char[] header, int headerLen) {
		try {
			port = new SerialReader(owner, baud_rate, listener, header, headerLen);
		} catch (Exception e) {
			// TODO Implement error handler for serial panel
		}
	}
	
	public void closePort() {
		port.close();
	}
}