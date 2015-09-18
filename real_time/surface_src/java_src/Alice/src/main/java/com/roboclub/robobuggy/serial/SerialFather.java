package com.roboclub.robobuggy.serial;

import gnu.io.CommPortIdentifier;
import gnu.io.PortInUseException;
import gnu.io.SerialPort;
import java.io.IOException;
import gnu.io.SerialPortEventListener;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Enumeration;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;
import com.roboclub.robobuggy.ros.Node;

// Properly initializes the serial things
public class SerialFather implements Node {

	List<SerialNode> children;


	public SerialFather(List<SerialNode> lsn) {
		Set<Integer> baudrates = new TreeSet<Integer>();
		int min_data_size = 1000000000; 
		
		for(SerialNode sn : lsn) {
			baudrates.add(sn.baudRate());
			if(sn.matchDataMinSize() < min_data_size) {
				min_data_size = sn.matchDataMinSize();
			}
		}
	}
	
	//TODO test this function
	private boolean isCorrectPort(SerialNode sn, InputStream input,String header,int bytesToRead ) throws IOException{
		byte[] buffer = new byte[bytesToRead];
		input.read(buffer, 0, bytesToRead);
		return sn.peel(buffer, 0,bytesToRead) > 1;
	}
	
	// TODO register connect/disconnect events
	
	// TODO do for all baud rates
	private void findPort(int baudrate, String header, String owner, SerialNode sn) {
		SerialPort port;
		CommPortIdentifier port_id;
		int TIMEOUT = 0;
		int BUFFER_SIZE = 512;
		InputStream input;
		OutputStream output;
		int index = 0;
		boolean connected;
		char[] inputBuffer;
		
		Enumeration<CommPortIdentifier> port_list = CommPortIdentifier
				.getPortIdentifiers();

		while (port_list.hasMoreElements()) {
			port_id = (CommPortIdentifier) port_list.nextElement();

			if (port_id.getPortType() != CommPortIdentifier.PORT_SERIAL) {
				continue;
			}
		
			System.out.println("Testing Port: " + port_id.getName());
			try {
				port = (SerialPort) port_id.open(owner, TIMEOUT);
				port.setInputBufferSize(BUFFER_SIZE);

				port.setSerialPortParams(baudrate, SerialPort.DATABITS_8,
						SerialPort.STOPBITS_1, SerialPort.PARITY_NONE);

				input = port.getInputStream();
				output = port.getOutputStream();

				// TODO wire through message length
				if (isCorrectPort(sn,input,header,512)){
					port.notifyOnDataAvailable(true);

					inputBuffer = new char[BUFFER_SIZE];
					index = 0;
					connected = true;
					//TODO fix the event Listener
					port.addEventListener((SerialPortEventListener) this);
					System.out.println("Connected to port: "
							+ port.getName());
					return;
				}

				port.close();
				input.close();
				output.close();
			} catch (PortInUseException e) {
			} catch (Exception e) {
				e.printStackTrace();
			
			}
		}
		System.out.println("Failed to connect for " + owner);
	}

	private boolean isCorrectPort(InputStream input, String header, int i) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean shutdown() {
		// TODO Auto-generated method stub
		return false;
	}


}
