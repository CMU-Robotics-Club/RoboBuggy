package com.roboclub.robobuggy.sensors;

/**
 * 
 * @author Trevor Decker 
 *
 * @version 0.5
 * 
 * CHANGELOG: NONE
 * 
 * DESCRIPTION: TODO
 */

public interface Sensor {
	SensorState getState();
	
	boolean isConnected();

	long timeOfLastUpdate();   // returns the last time that a message was parssed sucsssfully or that an error occured 

	boolean close();

	SensorType getSensorType();

	// Note that sensors are expected to publish to the channel
	//  that they have passed in.
	// TODO remove this. 
	void publish();
}
