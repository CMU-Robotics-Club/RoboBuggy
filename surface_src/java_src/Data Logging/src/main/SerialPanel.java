package main;

import javax.swing.JPanel;

import serial.SerialReader;

abstract class SerialPanel extends JPanel {
	private SerialReader port;
	
	public SerialPanel(String port_name, String owner, int baud_rate) {
		try {
			port = new SerialReader(port_name, owner, baud_rate);
		} catch (Exception e) {
			// TODO Implement error handler for serial panel
		}
	}
}