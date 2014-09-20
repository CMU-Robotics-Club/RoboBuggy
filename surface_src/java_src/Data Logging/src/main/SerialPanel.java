package main;

import javax.swing.JPanel;

import serial.SerialListener;
import serial.SerialReader;

abstract class SerialPanel extends JPanel {
	private static final long serialVersionUID = 8010633614957618540L;
	
	private SerialReader port;
	
	public SerialPanel(String owner, int baud_rate, char[] header,
			int headerLen) throws Exception {
		port = new SerialReader(owner, baud_rate, header, headerLen);
	}
	
	public void closePort() {
		port.close();
	}
	
	public void addListener(SerialListener listener) {
		port.addListener(listener);
	}
}