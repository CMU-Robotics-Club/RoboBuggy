package com.roboclub.robobuggy.sensors;

import java.io.IOException;
import java.io.InputStream;
import com.roboclub.robobuggy.messages.StateMessage;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.serial.SerialConnection;
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
	protected static final char ENC_RESET = (char)0;
	protected static final char ENC_TICK = (char)1;
	protected static final char ENC_TIME = (char)2;
	protected static final char STEERING = (char)20;
	protected static final char BRAKE = (char)21;
	protected static final char AUTO = (char)22;
	protected static final char BATTERY = (char)23;
	protected static final char ERROR = (char)254;
	protected static final char MSG_ID = (char)255;
	
	protected Arduino(SensorChannel sensor, String type) {
		super("Arduino-"+type, BAUDRATE, null, sensor.getRstPath());
		msgPub = new Publisher(sensor.getMsgPath());
		statePub = new Publisher(sensor.getStatePath());
		
		statePub.publish(new StateMessage(this.currState));
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
	
	byte[] bigBuffer = new byte[1024];
	@Override
	public void serialEvent(SerialPortEvent event) {
		int num_read = -1;
		switch (event.getEventType()) {
		case SerialPortEvent.DATA_AVAILABLE:
			try {
				num_read = input.read(bigBuffer);
			} catch (Exception e) {
				if (this.currState != SensorState.FAULT) {
					this.currState = SensorState.FAULT;
					statePub.publish(new StateMessage(this.currState));
				}
				System.out.println(this.getName() + " exception!");
				e.printStackTrace();
			}
			for(int i = 0; i < num_read; i++) {
				char data = (char) bigBuffer[i];
				inputBuffer[index++] = data;
				
				switch (state) {
				case 0:
					if (validId(data))
						state++;
					else
						index = 0;
					break;
				case 1:
					if (index >= MSG_LEN) {
						if (data == '\n')
							publish();
						index = 0;
						state = 0;
				}
					break;
				}
			}
		default:
			break;
		}
	}

	@Override
	protected boolean isCorrectPort(InputStream input, String header,
			int message_len) {
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
					if (data == '\n') return true; //TODO remove for real state++;
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
		return currState;
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
		if (connected) {
			
		}
		
		return false;
	}
	
	@Override
	public SensorType getSensorType() {
		return sensorType;
	}
}
