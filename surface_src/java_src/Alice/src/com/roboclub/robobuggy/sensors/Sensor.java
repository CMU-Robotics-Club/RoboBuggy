package com.roboclub.robobuggy.sensors;

import com.roboclub.robobuggy.serial.SerialListener;
import com.roboclub.robobuggy.serial.SerialReader;

public interface Sensor {
	SensorState getState();
	boolean isConnected();
	long timeOfLastUpdate();   // returns the last time that a message was parssed sucsssfully or that an error occured 
	boolean close();
	boolean reset();
	SensorType getSensorType();
}
