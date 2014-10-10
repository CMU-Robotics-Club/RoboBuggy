package com.roboclub.robobuggy.serial;

public class SerialConnection {
	protected SerialReader port;
	
	public SerialConnection(String owner, int baud_rate, String header) {
		port = new SerialReader(owner, baud_rate, header);
		
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
