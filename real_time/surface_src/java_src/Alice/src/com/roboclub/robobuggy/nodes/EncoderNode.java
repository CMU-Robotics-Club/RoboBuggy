package com.roboclub.robobuggy.nodes;

import java.util.Date;

import gnu.io.SerialPort;

import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.StateMessage;
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.sensors.SensorState;
import com.roboclub.robobuggy.serial.RBPair;
import com.roboclub.robobuggy.serial.RBSerial;
import com.roboclub.robobuggy.serial.RBSerialMessage;
import com.roboclub.robobuggy.serial.SerialNode;

/**
 * @author Matt Sebek
 * @author Kevin Brennan
 *
 * CHANGELOG: NONE
 * 
 * DESCRIPTION: Potential replacement for previous serial-reading framework.
 */

public class EncoderNode extends SerialNode implements Node {
	private static final double TICKS_PER_REV = 7.0;
	
	// Measured as 2 feet. Though could be made more precise. 
	private static final double M_PER_REV = 0.61;

	private double totalDist = 0.0;
	private int encTicks = 0;
	private double distLast = 0.0;
	private Date timeLast = new Date();
	
	Publisher messagePub;
	Publisher statePub;

	public EncoderNode(SensorChannel sensor) {
		super("encoder");
		messagePub = new Publisher(sensor.getMsgPath());
		statePub = new Publisher(sensor.getStatePath());
		
		//statePub.publish(new StateMessage(this.currState));

	}

	public void setSerialPort(SerialPort sp) {
		super.setSerialPort(sp);
		statePub.publish(new StateMessage(SensorState.ON));
	}
	
	private void estimateVelocity() {
		Date currTime = new Date();
		double dist = ((double)(encTicks)/TICKS_PER_REV) * M_PER_REV;
		double velocity = (dist)/ ((currTime.getTime() - timeLast.getTime()) * 1000);
		timeLast = currTime;
		totalDist = dist;
		messagePub.publish(new EncoderMeasurement(totalDist, velocity));
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
		// The Encoder sends 3 types of messages
		//  - Encoder ticks since last message (keep)
		//  - Number of ticks since last reset
		//  - Timestamp since reset
		RBPair rbp = RBSerial.peel(buffer, start, bytes_available);
		switch(rbp.getNumberOfBytesRead()) {
			case 0: return 0;
			case 1: return 1;
			case 6: break;
			default: {
				System.out.println("HOW DID NOT A SIX GET HERE");
			}
		
		}
		
		RBSerialMessage message = rbp.getMessage();
		if(message.getHeaderByte() == RBSerialMessage.ENC_TICK) {
			// This is a delta-distance! Do a thing!
			encTicks += message.getDataWord();
			estimateVelocity();
		}
		
		
		return 6;
	}
	
	
}
