package com.roboclub.robobuggy.main;

import java.awt.Color;
import java.awt.Font;

import javax.swing.BorderFactory;
import javax.swing.JLabel;
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
			System.out.println("Unable to connect to port for: " + owner);
		}
		
		if (port == null || !port.isConnected()) {
			System.out.println("No Serial Connection for: " + owner);
			JLabel msg = new JLabel("No Serial Connection for: " + owner);
			msg.setFont(new Font("sanserif",Font.PLAIN,20));
			msg.setForeground(Color.RED);
			this.add(msg);
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