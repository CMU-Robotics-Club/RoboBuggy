package com.roboclub.robobuggy.sensors;

import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.serial.Arduino;

public class Encoder implements Sensor {

	// Set up publishers
	private Publisher encoderPub = new Publisher("/sensor/encoder");
	
	public Encoder(String publishPath) 
	{
		Arduino.getInstance(); //makes sure that the arduino has been started 
	}
	
	public void updated()
	{
		double dist = Arduino.getInstance().getDist();
		double velocity = Arduino.getInstance().getVelocity();
		encoderPub.publish(new EncoderMeasurement(dist, velocity));
	}
	
	
	public boolean close()
	{
		//TODO 
		return false;
	}

}
