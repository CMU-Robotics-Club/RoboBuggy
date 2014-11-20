package com.roboclub.robobuggy.serial;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.*;

import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.sensors.SensorState;
import com.roboclub.robobuggy.sensors.SensorType;

import gnu.io.*;

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
public abstract class SerialConnection implements SerialPortEventListener {
	protected static final int TIMEOUT = 2000;
	protected static final int BUFFER_SIZE = 128;
	
	protected SerialPort port;
	protected Enumeration<CommPortIdentifier> port_list;
	protected CommPortIdentifier port_id;
	protected boolean connected;
	
	protected InputStream input;
	protected OutputStream output;
	protected char[] inputBuffer;
	protected int index;
	protected int state;
	
	protected long lastUpdateTime;
	protected SensorState currentState;
	protected SensorType thisSensorType;
	protected Publisher publisher;
	
	@SuppressWarnings("unchecked")
	protected SerialConnection(String owner, int baud_rate, String header) {
		port_list = CommPortIdentifier.getPortIdentifiers();
		connected = false;
		currentState = SensorState.DISCONNECTED;
		lastUpdateTime = 0;
		
		while (port_list.hasMoreElements() ) {
			port_id = (CommPortIdentifier)port_list.nextElement();
			
			if (port_id.getPortType() == CommPortIdentifier.PORT_SERIAL) {
				System.out.println("Testing Port: "+port_id.getName());
				try {
					port = (SerialPort)port_id.open(owner, TIMEOUT);
					port.setInputBufferSize(BUFFER_SIZE);
					
					port.setSerialPortParams( baud_rate,
							SerialPort.DATABITS_8,
			                SerialPort.STOPBITS_1,
			                SerialPort.PARITY_NONE );
					
					input = port.getInputStream();
					output = port.getOutputStream();
					
					if ( isCorrectPort(input, header) )	 {
						port.notifyOnDataAvailable(true);
						
						inputBuffer = new char[BUFFER_SIZE];
						index = 0;
						connected = true;
						currentState = SensorState.ON;
						port.addEventListener(this);
						System.out.println("Connected to port: " + this.getName());
						return;
					}
					
					port.close();
					input.close();
					output.close();
				} catch (PortInUseException e) {
					//TODO pass through for now
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
		}
		System.out.println("Failed to connect for " + owner);
	}
	
	protected boolean isCorrectPort(InputStream input, String header) {
		int j = 0;
		char data;
		for (int i = 0; i < 2*BUFFER_SIZE; i++) {
			try {
				data = (char)input.read();
			} catch (IOException e) {
				continue;
			}
			
			if (data == header.charAt(j)) j++;
			else j = 0;
			
			if (j == header.length()) return true;
		}
		
		return false;
	}
	
	public abstract void serialEvent(SerialPortEvent event);
	
	protected abstract void serialWrite(byte[] data);
	
	protected abstract void serialWrite(String data); 
	
	public String getName() {
		if (this.connected) {
			return this.port_id.getName();
		}
		
		return "";
	}
}
