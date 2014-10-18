package com.roboclub.robobuggy.serial;

/**
 * 
 * @author Kevin Brennan 
 *
 * @version 0.5
 * 
 * CHANGELOG: NONE
 * 
 * DESCRIPTION: TODO
 */

public class SerialConnection {
	protected SerialReader port;
	
	public SerialConnection(String owner, int baud_rate, String header, Integer ardType) {
		port = new SerialReader(owner, baud_rate, header, ardType);
		
		if (!port.isConnected()) {
			System.out.println("Failed to Create Connection for: " + owner);
		} else {
			System.out.println("Connected to " + port.getName() + " for: " + owner);
		}
	}
	
	public boolean close() {
		if (port != null && port.isConnected()) port.close();
		return true;
	}
	
	protected void addListener(SerialListener listener) {
		if (null != port) {
			port.addListener(listener);
		}
	}
	
	public boolean isConnected() {
		return (this.port != null && this.port.isConnected());
	}
}
