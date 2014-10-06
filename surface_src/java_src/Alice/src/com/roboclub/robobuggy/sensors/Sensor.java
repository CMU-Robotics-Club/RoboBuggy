package com.roboclub.robobuggy.sensors;

import com.roboclub.robobuggy.serial.SerialListener;
import com.roboclub.robobuggy.serial.SerialReader;

public class Sensor {
	protected SerialReader port;
	
	public Sensor(String owner, int baud_rate, String header) {
		port = new SerialReader(owner, baud_rate, header);
		
		if (!port.isConnected()) {
			System.out.println("Failed to Create Connection for: " + owner);
		} else {
			System.out.println("Connected to " + port.getName() + " for: " + owner);
		}
	}
	
	public void close() {
		if (port != null && port.isConnected()) port.close();
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
