package com.roboclub.robobuggy.sensors;

import com.roboclub.robobuggy.main.Robot;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.serial.Arduino;
import com.roboclub.robobuggy.serial.SerialEvent;
import com.roboclub.robobuggy.serial.SerialListener;

public class Encoder extends Arduino {
	public static final char ENC_TIME = (char)0x02;
	public static final char ENC_RESET = (char)0x00;
	public static final char ENC_TICK = (char)0x01;
	
	private static final long TICKS_PER_REV = 5;
	private static final double M_PER_REV = 5.0;
	
	private int encReset;
	private int encTickLast;
	private int encTime;
	private double dist;
	private double velocity;

	// Set up publishers
	private Publisher encoderPub;
	private SensorType thisSensorType;
	
	public Encoder() 
	{
		super("Encoder", 0);
		
		if (port != null && port.isConnected()) super.addListener(new EncoderListener());
		
		encoderPub = new Publisher("/sensor/encoder");
	}
	
	public boolean reset(){
		//TODO
		return false;
	}
	
	//is run after the encoder gets new data to update internal sate variables 
	public void updated()
	{
		lastUpdateTime = System.currentTimeMillis();
		currentSensorState = SensorState.ON; //TODO fix this 
		
		dist = ((double)(encTickLast - encReset)/TICKS_PER_REV) / M_PER_REV;
		velocity = dist / encTime;	
		
		encoderPub.publish(new EncoderMeasurement(dist, velocity));
	}
	
	public long timeOfLastUpdate(){
		return lastUpdateTime;
	}
	
	/* Methods for reading from Serial */
	public static boolean validId(char value) {
		switch (value) {
			case ENC_TIME:
			case ENC_RESET:
			case ENC_TICK:
			case Arduino.ERROR:
				return true;
			default:
				return false;
		}
	}
	
	public class EncoderListener implements SerialListener {
		@Override
		public void onEvent(SerialEvent event) {
			char[] tmp = event.getBuffer();
			if (event.getLength() != MSG_LEN) return;
			
			switch (tmp[0]) {
			case ENC_TIME:
				encTime = parseInt(tmp[1], tmp[2], tmp[3], tmp[4]);
				break;
			case ENC_RESET:
				encReset = parseInt(tmp[1], tmp[2], tmp[3], tmp[4]);
				break;
			case ENC_TICK:
				encTickLast = parseInt(tmp[1], tmp[2], tmp[3], tmp[4]);
				System.out.println("Time: " + encTime + ", Reset: " + encReset + ", Ticks: " + encTickLast);
				Robot.UpdateEnc(encTime, encReset, encTickLast);
				break;
			case ERROR:
				Robot.UpdateError(parseInt(tmp[1], tmp[2], tmp[3], tmp[4]));
				break;
			default:
				return;
			}
		}
	}
	@Override
	public SensorType getSensorType() {
		return thisSensorType;
	}

}
