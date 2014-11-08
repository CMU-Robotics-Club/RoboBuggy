package com.roboclub.robobuggy.sensors;

import com.roboclub.robobuggy.main.Robot;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.serial.Arduino;

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
	
	public Encoder() {
		super("Encoder", "/sensor/encoder");
		thisSensorType = SensorType.ENCODER;
	}
	
	public int getTicks(){
		return encTicks;
	}
	
	//is run after the encoder gets new data to update internal sate variables 
	public void updated()
	{
		lastUpdateTime = System.currentTimeMillis();
		currentState = SensorState.ON; //TODO fix this 
		
		dist = ((double)(encTicks - encReset)/TICKS_PER_REV) / M_PER_REV;
		velocity = dist / encTime;	
		
		publisher.publish(new EncoderMeasurement(dist, velocity));
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
		currentState = SensorState.ON;
		lastUpdateTime = System.currentTimeMillis();
		
		try {
			switch (inputBuffer[0]) {
			case ENC_TIME:
				encTime = parseInt(inputBuffer[1], inputBuffer[2],
						inputBuffer[3], inputBuffer[4]);
				break;
			case ENC_RESET:
				encReset = parseInt(inputBuffer[1], inputBuffer[2], 
						inputBuffer[3], inputBuffer[4]);
				break;
			case ENC_TICK:
				encTicks = parseInt(inputBuffer[1], inputBuffer[2], 
						inputBuffer[3], inputBuffer[4]);
				Robot.UpdateEnc(encTime, encReset, encTicks);
				//System.out.println("Time: " + encTime + " Reset: " + encReset + " Ticks: " + encTicks);
				break;
			case ERROR:
				Robot.UpdateError(parseInt(inputBuffer[1], inputBuffer[2], 
						inputBuffer[3], inputBuffer[4]));
				break;
			}
		} catch (Exception e) {
			System.out.println("Encoder Exception on port: " + this.getName());
		}
	}
}
