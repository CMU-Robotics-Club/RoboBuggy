package com.roboclub.robobuggy.ui;

import javax.swing.JPanel;

public class ImuPanel extends SerialPanel {
	private static final long serialVersionUID = -929040896215455343L;

	public ImuPanel(String port_name, int baud_rate) throws Exception {
		super(port_name, "ARDUINO", 9600);
		
	}
}