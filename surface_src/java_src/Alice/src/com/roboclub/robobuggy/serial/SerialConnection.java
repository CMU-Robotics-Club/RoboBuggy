package com.roboclub.robobuggy.serial;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.*;

import javax.management.RuntimeErrorException;

import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.Subscriber;
import com.roboclub.robobuggy.sensors.SensorState;
import com.roboclub.robobuggy.sensors.SensorType;

import gnu.io.*;

/**
 * 
 * @author Kevin Brennan
 *
 * @version 0.5
 * 
 *          CHANGELOG: NONE
 * 
 *          DESCRIPTION: TODO
 */
public abstract class SerialConnection implements SerialPortEventListener {
	protected static final int TIMEOUT = 2000;
	protected static final int BUFFER_SIZE = 1028; // 128; TODO need to find
													// correct number

	protected SerialPort port;
	protected CommPortIdentifier port_id;
	protected boolean connected;

	protected InputStream input;
	protected OutputStream output;
	protected char[] inputBuffer;
	protected int index;
	protected int state;

	protected long lastUpdateTime;
	protected SensorState currState;
	protected SensorType sensorType;
	protected Publisher msgPub;
	protected Publisher statePub;
	protected Subscriber ctrlSub;

	protected SerialConnection(String owner, int baudrate, String header,
			String ctrlPath) {
		connected = false;
		currState = SensorState.DISCONNECTED;
		lastUpdateTime = 0;
		ctrlSub = new Subscriber(ctrlPath, new ResetListener(this));

		findPort(baudrate, header, owner);
	}

	@SuppressWarnings("unchecked")
	private void findPort(int baudrate, String header, String owner) {
		Enumeration<CommPortIdentifier> port_list = CommPortIdentifier
				.getPortIdentifiers();

		while (port_list.hasMoreElements()) {
			port_id = (CommPortIdentifier) port_list.nextElement();

			if (port_id.getPortType() == CommPortIdentifier.PORT_SERIAL) {
				System.out.println("Testing Port: " + port_id.getName());
				try {
					port = (SerialPort) port_id.open(owner, TIMEOUT);
					port.setInputBufferSize(BUFFER_SIZE);

					port.setSerialPortParams(baudrate, SerialPort.DATABITS_8,
							SerialPort.STOPBITS_1, SerialPort.PARITY_NONE);

					input = port.getInputStream();
					output = port.getOutputStream();

					// TODO wire through message length
					if (isCorrectPort(input, header, 512)) {
						port.notifyOnDataAvailable(true);

						inputBuffer = new char[BUFFER_SIZE];
						index = 0;
						connected = true;
						currState = SensorState.ON;
						port.addEventListener(this);
						System.out.println("Connected to port: "
								+ this.getName());
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
		}
		System.out.println("Failed to connect for " + owner);
	}

	// TODO repeated reading of serial stuff is inefficient, but not the end of
	// the world.
	protected boolean isCorrectPort(InputStream input, String header,
			int message_length) {

		byte[] temp = new byte[2 * message_length];
		int num_read = 0;
		// Block until we have enough buffered bytes to decide
		// which is which.
		while (num_read < 2 * message_length) {
			try {
				num_read += input.read(temp, num_read, 2 * message_length
						- num_read);
			} catch (IOException e) {
				e.printStackTrace();
				return false;
			}
		}
		String s = new String(temp);
		char[] buf = s.toCharArray();

		// Check for a prefix
		for (int i = 0; i < 2 * message_length; i++) {
			if (is_prefix(buf, header, i) == true) {
				return true;
			}
		}

		return false;
	}

	// returns true iff input contains header starting at start_location.
	private boolean is_prefix(char[] input, String header, int start_location) {
		if (input.length < start_location + header.length()) {
			return false;
		}

		for (int i = start_location; i < start_location + header.length(); i++) {
			if (input[i] != header.charAt(i - start_location)) {
				return false;
			}
		}
		return true;
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

	public void disconnect() {
		if (connected) {
			port.removeEventListener();
			port.close();
			try {
				input.close();
			} catch (IOException e) {
				e.printStackTrace();
			}
			try {
				output.close();
			} catch (IOException e) {
				e.printStackTrace();
			}
			connected = false;
		}

		currState = SensorState.DISCONNECTED;
	}

	private class ResetListener implements MessageListener {
		private SerialConnection parent;

		public ResetListener(SerialConnection parent) {
			this.parent = parent;
		}

		@Override
		public void actionPerformed(String topicName, Message m) {
			disconnect();

			// Attempt to reconnect to port
			try {
				port = (SerialPort) port_id.open(port_id.getCurrentOwner(),
						TIMEOUT);
				port.setInputBufferSize(BUFFER_SIZE);

				port.setSerialPortParams(port.getBaudRate(),
						SerialPort.DATABITS_8, SerialPort.STOPBITS_1,
						SerialPort.PARITY_NONE);

				input = port.getInputStream();
				output = port.getOutputStream();
				port.addEventListener(this.parent);

				connected = true;
				currState = SensorState.ON;
			} catch (Exception e) {
				System.out.println("Failed to reconnect "
						+ port_id.getCurrentOwner());
				e.printStackTrace();
			}
		}
	}
}
