package com.roboclub.robobuggy.serial;

import com.roboclub.robobuggy.sensors.Sensor;
import com.roboclub.robobuggy.sensors.SensorState;
import com.roboclub.robobuggy.sensors.SensorType;

public class Arduino extends SerialConnection implements Sensor {
	public static final int MSG_LEN = 6;
	public static final char ERROR = (char)0xFF;
	protected static final int BAUDRATE = 9600;
	
	protected long lastUpdateTime;
	protected SensorState currentSensorState;
	
	public Arduino(String type, int ardType) {
		super("Arduino-" + type, BAUDRATE, null, ardType);
	}
	
	public boolean close()
	{
		//TODO 
		return false;
	}

	@Override
	public SensorState getState() {
		return currentSensorState;
	}

	@Override
	public boolean isConnected() {
		if(currentSensorState == SensorState.AVILABLE){
			return true;
		}
		
		if(currentSensorState == SensorState.ERROR){
			return true;
		}
		
		if(currentSensorState == SensorState.ON){
			return true;
		}
		return false;
	}

	@Override
	public long timeOfLastUpdate() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public boolean reset() {
		// TODO Auto-generated method stub
		return false;
	}
	
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
	public SensorType getSensorType() {
		return null;
	}
}
