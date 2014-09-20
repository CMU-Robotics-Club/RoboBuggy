package com.roboclub.robobuggy.ui;

import javax.swing.JPanel;

//import serial.SerialReader;

public abstract class SerialPanel extends JPanel {

	private static final long serialVersionUID = 7509351582829554466L;
	
	public SerialPanel(String port_name, String owner, int baud_rate) {
		try {
		} catch (Exception e) {
			// TODO Implement error handler for serial panel
		}
	}
}