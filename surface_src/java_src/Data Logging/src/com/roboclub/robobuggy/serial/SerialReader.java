package com.roboclub.robobuggy.serial;

import java.io.InputStream;
import java.io.OutputStream;
import java.util.*;

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
	//private char[] outputBuffer;
	private int index;
	
	private ArrayList<SerialListener> listeners;
	
	@SuppressWarnings("unchecked")
	public SerialReader(String owner, int baud_rate,
			char[] header, int headerLen) throws Exception {
		listeners = new ArrayList<SerialListener>();
		port_list = CommPortIdentifier.getPortIdentifiers();
		connected = false;
		
		while (port_list.hasMoreElements() ) {
			port_id = (CommPortIdentifier)port_list.nextElement();
			
			if (port_id.getPortType() == CommPortIdentifier.PORT_SERIAL) {
				if (!port_id.isCurrentlyOwned() ) {
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
						if (inByte != '?') msg[numRead++] = inByte;
					}
				
					passed = checkHeader(msg, header, numRead, headerLen);
					
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
				}
			}
		}
	}
	
	private boolean checkHeader(char[] msg, char[] header, int numRead, int headerLen) {
		int start = 0;
		
		while ( start < numRead ) {
			for (int i = 0; i < headerLen; i++) {
				if (msg[i + start] != header[i]) {
					break;
				}
				else if (i == (headerLen - 1)) return true;
			}
			
			start++;
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
					index = 0;
				}
			} catch (Exception e) {
				listeners.clear();
				port.close();
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
			System.exit(-1);
		}
	}
	
	public boolean isConnected() {
		return this.connected;
	}
}