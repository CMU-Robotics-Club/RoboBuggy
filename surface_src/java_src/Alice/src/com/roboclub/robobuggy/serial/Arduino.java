package com.roboclub.robobuggy.serial;

import java.io.IOException;
import java.io.InputStream;

import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.sensors.Sensor;
import com.roboclub.robobuggy.sensors.SensorState;
import com.roboclub.robobuggy.sensors.SensorType;
import gnu.io.SerialPortEvent;

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

public abstract class Arduino extends SerialConnection implements Sensor {
	protected static final int BAUDRATE = 9600;
	protected static final int MSG_LEN = 6;
	protected static final char ENC_RESET = (char)0x00;
	protected static final char ENC_TICK = (char)0x01;
	protected static final char ENC_TIME = (char)0x02;
	protected static final char STEERING = (char)0x0A;
	protected static final char BRAKE = (char)0x0B;
	protected static final char ERROR = (char)0xFE;
	protected static final char MSG_ID = (char)0xFF;
	
	protected long lastUpdateTime;
	protected SensorState currentState;
	
	protected Publisher publisher;
	private SensorType thisSensorType;
	
	protected Arduino(String type, String path) {
		super("Arduino-"+type, BAUDRATE, null);
		publisher = new Publisher(path);
	}

	protected abstract boolean validId(char id);
	
	protected int parseInt(char x0, char x1, char x2, char x3) {
		int val = 0;
		val |= x0;
		val = val << 0x8;
		val |= x1;
		val = val << 0x8;
		val |= x2;
		val = val << 0x8;
		val |= x3;
		
		return val;
	}
	
	@Override
	public void serialEvent(SerialPortEvent event) {
		switch (event.getEventType()) {
		case SerialPortEvent.DATA_AVAILABLE:
			try {
				char data = (char)input.read();
				
				inputBuffer[index++] = data;
				
				switch (state) {
				case 0:
					if (validId(data)) state++;
					else index = 0;
					break;
				case 1:
					if (index >= MSG_LEN) {
						if (data == '\n') publish();
						index = 0;
						state = 0;
					}
					break;
				}
			} catch (Exception e) {
				System.out.println("Exception in port: " + this.getName());
			}
		}
	}

	@Override
	protected boolean isCorrectPort(InputStream input, String header) {
		char data;
		int state = 0, test = 0;
			
		for (int i = 0; i < 2*BUFFER_SIZE; i++) {
			try {
				data = (char)input.read();
			} catch (IOException e) {
				continue;
			}
			
			switch (state) {
			case 0:
				if (validId(data)) {
					state++;
					test++;
				}
				break;
			case 1:
				test++;
				if (test >= MSG_LEN) {
					if (data == '\n') state++;
					else state = 0;
					
					test = 0;
				}
				break;
			case 2:
				if (validId(data)) {
					state++;
					test++;
				} else {
					state = 0;
					test = 0;
				}
				break;
			case 3:
				test++;
				if (test >= MSG_LEN) {
					if (data == '\n') return true;
					else state = 0;
					
					test = 0;
				}
				break;
			}
		}
		
		return false;
	}
	
	@Override
	protected void serialWrite(byte[] data) {
		try {
			output.write(data);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	@Override
	protected void serialWrite(String data) {
		try {
			output.write(data.getBytes());
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	@Override
	public SensorState getState() {
		return currentState;
	}

	@Override
	public boolean isConnected() {
		return connected;
	}

	@Override
	public long timeOfLastUpdate() {
		return lastUpdateTime;
	}

	@Override
	public boolean close() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean reset() {
		// TODO Auto-generated method stub
		return false;
	}
	
	@Override
	public SensorType getSensorType() {
		return thisSensorType;
	}
}
