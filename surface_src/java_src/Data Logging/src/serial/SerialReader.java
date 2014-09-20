package serial;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.*;

import gnu.io.*;

public class SerialReader implements SerialPortEventListener {
	private static final int TIMEOUT = 2000;
	private static final int BUFFER_SIZE = 256;
	
	private SerialPort port;
	private Enumeration port_list;
	private CommPortIdentifier port_id;
	
	private InputStream input;
	private OutputStream output;
	
	private char[] inputBuffer;
	private char[] outputBuffer;
	private int index;
	
	private ArrayList<SerialListener> listeners;
	
	public SerialReader(String port_name, String owner, int baud_rate) throws Exception {
		port_list = CommPortIdentifier.getPortIdentifiers();
		
		while (port_list.hasMoreElements() ) {
			port_id = (CommPortIdentifier)port_list.nextElement();
			
			if (port_id.getPortType() == CommPortIdentifier.PORT_SERIAL) {
				if (port_id.getName().equalsIgnoreCase( port_name ) && !port_id.isCurrentlyOwned() ) {
					try {
						port = (SerialPort)port_id.open(owner, TIMEOUT);
						
						port.setSerialPortParams( baud_rate,
								SerialPort.DATABITS_8,
				                SerialPort.STOPBITS_1,
				                SerialPort.PARITY_NONE );
						
						port.notifyOnDataAvailable(true);
						
						inputBuffer = new char[BUFFER_SIZE];
						outputBuffer = new char[BUFFER_SIZE];
						index = 0;
						
						port.addEventListener(this);
						input = port.getInputStream();
						output = port.getOutputStream();		
					} catch ( Exception e ) {
						throw new Exception( "Unable to open serial port: " + port_name );
					}
				}
			}
		}
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
				// TODO Add error handler for serial port
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
				listener.sendData(inputBuffer, index);
			}
		}
	}
}