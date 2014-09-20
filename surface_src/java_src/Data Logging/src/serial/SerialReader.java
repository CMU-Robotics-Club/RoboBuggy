package serial;

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
	
	private InputStream input;
	private OutputStream output;
	
	private char[] inputBuffer;
	private char[] outputBuffer;
	private int index;
	
	private ArrayList<SerialListener> listeners;
	
	@SuppressWarnings("unchecked")
	public SerialReader(String owner, int baud_rate,
			SerialListener listener, char[] header, int headerLen) throws Exception {
		listeners = new ArrayList<SerialListener>();
		port_list = CommPortIdentifier.getPortIdentifiers();
		
		while (port_list.hasMoreElements() ) {
			port_id = (CommPortIdentifier)port_list.nextElement();
			
			if (port_id.getPortType() == CommPortIdentifier.PORT_SERIAL) {
				if (!port_id.isCurrentlyOwned() ) {
					try {
						port = (SerialPort)port_id.open(owner, TIMEOUT);
						
						port.setSerialPortParams( baud_rate,
								SerialPort.DATABITS_8,
				                SerialPort.STOPBITS_1,
				                SerialPort.PARITY_NONE );
						
						input = port.getInputStream();
						output = port.getOutputStream();
						
						char[] msg = new char[128];
						int numRead = 0;
						boolean passed = false;
						
						while (true) {
							char inByte = (char)input.read();
							msg[numRead++] = inByte;
							
							if (inByte == '\n') {
								passed = checkHeader(msg, header, numRead, headerLen);
								break;
							}
						}
						
						if ( passed )	 {
							port.notifyOnDataAvailable(true);
							
							inputBuffer = new char[BUFFER_SIZE];
							outputBuffer = new char[BUFFER_SIZE];
							index = 0;
							
							port.addEventListener(this);
							listeners.add(listener);
							break;
						}
						
						port.close();
					} catch ( Exception e ) {
						throw new Exception( "Unable to open serial port");
					}
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
				System.out.print(data);
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
	
	public void close() {
		port.close();
	}
}