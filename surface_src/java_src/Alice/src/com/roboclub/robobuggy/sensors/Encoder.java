package com.roboclub.robobuggy.sensors;

import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.serial.Arduino;

public class Encoder implements Sensor {
	private long lastUpdateTime;
	private SensorState currentSensorState;
	// Set up publishers
	private Publisher encoderPub = new Publisher("/sensor/encoder");
	private SensorType thisSensorType;
	
	public Encoder(String publishPath) 
	{
		Arduino.getInstance(); //makes sure that the arduino has been started 
	}
	
	public boolean reset(){
		//TODO
		return false;
	}
	
	//is run after the encoder gets new data to update internal sate variables 
	public void updated()
	{
		lastUpdateTime = System.currentTimeMillis();  
		double dist = Arduino.getInstance().getDist();
		double velocity = Arduino.getInstance().getVelocity();
		currentSensorState = SensorState.ON; //TODO fix this 
		encoderPub.publish(new EncoderMeasurement(dist, velocity));
	}
	
	public long timeOfLastUpdate(){
		return lastUpdateTime;
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
	public SensorType getSensorType() {
		return thisSensorType;
	}

}
