package com.roboclub.robobuggy.nodes;

import gnu.io.SerialPort;

import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.StateMessage;
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.sensors.SensorState;
import com.roboclub.robobuggy.sensors.SensorType;
import com.roboclub.robobuggy.serial2.RBSerial;
import com.roboclub.robobuggy.serial2.SerialNode;
import com.roboclub.robobuggy.serial2.RBPair;

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

	Publisher messagePub;
	Publisher statePub;

	public EncoderNode2(SensorChannel sensor) {
		super("encoder");
		messagePub = new Publisher(sensor.getMsgPath());
		statePub = new Publisher(sensor.getStatePath());
		
		//statePub.publish(new StateMessage(this.currState));

	}

	public void setSerialPort(SerialPort sp) {
		super.setSerialPort(sp);
	}
	
	private void estimateVelocity() {
		double dist = ((double)(encTicks)/TICKS_PER_REV) / M_PER_REV;
		double velocity = (dist - distLast)/ (double)encTime;
		distLast = dist;
		messagePub.publish(new EncoderMeasurement(dist, velocity));
	}
	
	@Override
	public boolean matchDataSample(byte[] sample) {
		// Peel what ever is not a message
		
		// Check that whatever we give it is a message
		
		return true;
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
	public int peel(byte[] buffer, int start, int bytes_available) {
		
		RBPair rbp = RBSerial.peel(buffer, start, bytes_available);
		if(rbp.getNumber() == 0) {
			// Could not read a message....try again in one byte
			System.out.printf("corruption sigh");
			return 1;
		}
		System.out.println("goodread");
		return 6;
	}
	
	
}
