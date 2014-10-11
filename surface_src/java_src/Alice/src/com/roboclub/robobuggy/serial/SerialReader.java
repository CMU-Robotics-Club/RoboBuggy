package com.roboclub.robobuggy.serial;

import java.io.InputStream;
import java.io.OutputStream;
import java.util.*;

import com.roboclub.robobuggy.sensors.DriveActuator;
import com.roboclub.robobuggy.sensors.Encoder;

import gnu.io.*;

public class SerialReader implements SerialPortEventListener {
	private static final int TIMEOUT = 2000;
	private static final int BUFFER_SIZE = 256;
	
	private SerialPort port;
	private Enumeration<CommPortIdentifier> port_list;
	private CommPortIdentifier port_id;
	private boolean connected;
	
	private InputStream input;
	private OutputStream output;
	
	private char[] inputBuffer;
	private int index;
	
	private ArrayList<SerialListener> listeners;
	
	@SuppressWarnings("unchecked")
	public SerialReader(String owner, int baud_rate, String header, Integer ardType) {
		listeners = new ArrayList<SerialListener>();
		port_list = CommPortIdentifier.getPortIdentifiers();
		connected = false;
		
		while (port_list.hasMoreElements() ) {
			port_id = (CommPortIdentifier)port_list.nextElement();
			
			if (port_id.getPortType() == CommPortIdentifier.PORT_SERIAL) {
				if (!port_id.isCurrentlyOwned() ) {
					try {
						port = (SerialPort)port_id.open(owner, TIMEOUT);
						port.setInputBufferSize(BUFFER_SIZE);
						
						port.setSerialPortParams( baud_rate,
								SerialPort.DATABITS_8,
				                SerialPort.STOPBITS_1,
				                SerialPort.PARITY_NONE );
						
						input = port.getInputStream();
						output = port.getOutputStream();
						
						char[] msg = new char[BUFFER_SIZE];
						int numRead = 0;
						boolean passed = false;
						
						while (numRead < BUFFER_SIZE) {
							char inByte = (char)input.read();
							msg[numRead++] = inByte;
						}
					
						passed = checkHeader(msg, header, numRead, ardType);
						
						if ( passed )	 {
							port.notifyOnDataAvailable(true);
							
							inputBuffer = new char[BUFFER_SIZE];
							//outputBuffer = new char[BUFFER_SIZE];
							index = 0;
							connected = true;
							port.addEventListener(this);
							return;
						}
						
						port.close();
						input.close();
						output.close();
					}  catch (Exception e) {
						e.printStackTrace();
					}
				}
			}
		}
	}
	
	private boolean checkHeader(char[] msg, String header, int numRead, Integer ardType) {
		int start = 0;
		
		if (header != null) {
			while ( start < numRead ) {
				for (int i = 0; i < header.length(); i++) {
					if (msg[i + start] != header.charAt(i)) {
						break;
					}
					else if (i == (header.length() - 1)) return true;
				}
				
				start++;
			}
		} else {
			for (int i = 0; i < (numRead-Arduino.MSG_LEN); i++) {
				switch (ardType) {
				case 0:
					if (Encoder.validId(msg[i]) && msg[i+Arduino.MSG_LEN-1] == '\n') return true;
					break;
				case 1:
					if (DriveActuator.validId(msg[i]) && msg[i+Arduino.MSG_LEN-1] == '\n') return true;
					break;
				default:
				}
					
			}
		}
		
		return false;
	}
	
	public void serialEvent(SerialPortEvent event) {
		switch (event.getEventType()) {
		case SerialPortEvent.DATA_AVAILABLE:
			try {
				char data = (char)input.read();
				inputBuffer[index++] = data;
				
				if (data == '\n') {
					notifyListeners();					
					index = 0;
				}
				
				// Reset buffer index in overflow
				if (index == BUFFER_SIZE) {
					System.out.println("Overflow!");
					index = 0;
				}
			} catch (Exception e) {
				System.out.println("Exception, Why?");
				e.printStackTrace();
				return;
			}

			break;
		default:
			break;
		}
	}
	
	private void notifyListeners() {
		if (null != listeners && !listeners.isEmpty()) {
			for (SerialListener listener : listeners) {
				listener.onEvent(new SerialEvent(inputBuffer, index));
			}
		}
	}
	
	public void addListener(SerialListener listener) {
		listeners.add(listener);
	}
	
	public void close() {
		try {
			input.close();
			output.close();
			port.close();
		} catch (Exception e) {
			System.out.println("Failed to Close Port: " + this.getName());
		}
	}
	
	public boolean isConnected() {
		return this.connected;
	}
	
	public void serialWrite(byte[] data) {
		if (connected && output != null) {
			try {
				output.write(data);
				System.out.println("Wrote: " + data);
				//output.flush();
			} catch (Exception e) {
				System.out.println("Unable to write: " + data);
			}
		}
	} 
	
	public void serialWrite(String data) {
		if (connected && data != null && output != null) {
			try {
				output.write(data.getBytes());
				output.flush();
			} catch (Exception e) {
				System.out.println("Unable to write: " + data);
			}
		}
	}
	
	public String getName() {
		if (this.connected) {
			return this.port_id.getName();
		}
		
		return "";
	}
}