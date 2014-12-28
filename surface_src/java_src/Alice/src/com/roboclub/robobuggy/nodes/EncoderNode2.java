package com.roboclub.robobuggy.nodes;

import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.StateMessage;
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.sensors.SensorState;
import com.roboclub.robobuggy.sensors.SensorType;
import com.roboclub.robobuggy.serial2.RBSerial;
import com.roboclub.robobuggy.serial2.SerialNode;

/**
 * @author Matt Sebek
 * @author Kevin Brennan
 *
 * CHANGELOG: NONE
 * 
 * DESCRIPTION: Potential replacement for previous serial-reading framework.
 */

public class EncoderNode2 extends SerialNode implements Node {
	private static final long TICKS_PER_REV = 5;
	private static final double M_PER_REV = 5.0;
	private static final int MAX_TICKS = 0xFFFF;
	
	private int encReset;
	private int encTicks;
	private int encTime;
	private double distLast;

	Publisher encoderPub;
	Publisher statePub;

	RBSerial input_parser = new RBSerial();
	
	public EncoderNode2(SensorChannel sensor) {
		super(sensor, "Encoder");
		msgPub = new Publisher(sensor.getMsgPath());
		statePub = new Publisher(sensor.getStatePath());
		sensorType = SensorType.ENCODER;
		statePub.publish(new StateMessage(this.currState));

	}

	private void estimateVelocity() {
		double dist = ((double)(encTicks)/TICKS_PER_REV) / M_PER_REV;
		double velocity = (dist - distLast)/ (double)encTime;
		distLast = dist;
		msgPub.publish(new EncoderMeasurement(dist, velocity));
	}
	
	@Override
	public boolean matchDataSample(byte[] sample) {
		// Peel what ever is not a message
		
		// Check that whatever we give it is a message
		
		return false;
	}

	@Override
	public int matchDataMinSize() {
		return 2*RBSerial.MSG_LEN;
	}

	@Override
	public int baudRate() {
		return 9600;
	}

	@Override
	public void peel(byte[] buffer, int start, int bytes_available) {
		// TODO Auto-generated method stub
		
	}
	
	
}
