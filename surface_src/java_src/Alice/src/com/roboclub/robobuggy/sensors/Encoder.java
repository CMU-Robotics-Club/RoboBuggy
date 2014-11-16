package com.roboclub.robobuggy.sensors;

import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.StateMessage;
import com.roboclub.robobuggy.ros.SensorChannel;

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

public class Encoder extends Arduino {
	private static final long TICKS_PER_REV = 5;
	private static final double M_PER_REV = 5.0;
	
	private int encReset;
	private int encTicks;
	private int encTime;
	private double dist;
	private double velocity;
	
	public Encoder(SensorChannel sensor) {
		super(sensor, "Encoder");
		sensorType = SensorType.ENCODER;
	}
	
	public int getTicks(){
		return encTicks;
	}
	
	//is run after the encoder gets new data to update internal sate variables 
	public void updated()
	{
		lastUpdateTime = System.currentTimeMillis();
		currState = SensorState.ON; //TODO fix this 
		
		dist = ((double)(encTicks - encReset)/TICKS_PER_REV) / M_PER_REV;
		velocity = dist / encTime;	
		
		msgPub.publish(new EncoderMeasurement(dist, velocity));
	}
	
	/* Methods for reading from Serial */
	@Override
	public boolean validId(char value) {
		switch (value) {
			case ENC_TIME:
			case ENC_RESET:
			case ENC_TICK:
			case ERROR:
			case MSG_ID:
				return true;
			default:
				return false;
		}
	}
	
	@Override
	public void publish() {
		lastUpdateTime = System.currentTimeMillis();
		
		int value = parseInt(inputBuffer[1], inputBuffer[2],
				inputBuffer[3], inputBuffer[4]);
		try {
			switch (inputBuffer[0]) {
			case ENC_TIME:
				encTime = value;
				break;
			case ENC_RESET:
				encReset = value;
				break;
			case ENC_TICK:
				encTicks = value;
				// TODO update velocity estimation
				break;
			case ERROR:
				// TODO handle errors
				break;
			}
		} catch (Exception e) {
			System.out.println("Encoder Exception on port: " + this.getName());
			if (this.currState != SensorState.FAULT) {
				this.currState = SensorState.FAULT;
				statePub.publish(new StateMessage(this.currState));
			}
		}
		
		if (this.currState != SensorState.ON) {
			this.currState = SensorState.ON;
			statePub.publish(new StateMessage(this.currState));
		}
	}
	
	
}
