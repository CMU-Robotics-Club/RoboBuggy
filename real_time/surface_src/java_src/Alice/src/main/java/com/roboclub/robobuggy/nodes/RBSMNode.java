package com.roboclub.robobuggy.nodes;

import java.util.Date;

import gnu.io.SerialPort;

import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.StateMessage;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.sensors.SensorState;
import com.roboclub.robobuggy.serial.RBPair;
import com.roboclub.robobuggy.serial.RBSerial;
import com.roboclub.robobuggy.serial.RBSerialMessage;
import com.roboclub.robobuggy.serial.SerialNode;

/**
 * @author Trevor Decker
 * @author Matt Sebek
 * @author Kevin Brennan
 *
 * CHANGELOG: NONE
 * 
 * DESCRIPTION: node for talking to the the low level controller via rbsm
 */

public class RBSMNode extends SerialNode implements Node 
{
	private static final double TICKS_PER_REV = 7.0;
	
	// Measured as 2 feet. Though could be made more precise. 
	private static final double M_PER_REV = 0.61;
	
	/** Steering Angle Conversion Rate */
	private final int ARD_TO_DEG = 100;
	/** Steering Angle offset?? */
	private final int OFFSET = -200;


	// accumulated
	private int encTicks = 0;
	private int potValue = -1;
	
	// last state
	private double accDistLast = 0.0;
	private double instVelocityLast = 0.0;
	private Date timeLast = new Date();
	
	Publisher messagePub_enc;
	Publisher messagePub_pot;
	Publisher brakePub; 
	
	Publisher statePub_enc;
	Publisher statePub_pot;

	public RBSMNode(SensorChannel sensor_enc,SensorChannel sensor_pot) 
	{
		super("ENCODER");
		messagePub_enc = new Publisher(sensor_enc.getMsgPath());
		messagePub_pot = new Publisher(sensor_pot.getMsgPath());
		brakePub = new Publisher(SensorChannel.BRAKE.getMsgPath());

		statePub_enc = new Publisher(sensor_enc.getStatePath());
		statePub_pot = new Publisher(sensor_pot.getStatePath());

		
		statePub_enc.publish(new StateMessage(SensorState.DISCONNECTED));
		statePub_pot.publish(new StateMessage(SensorState.DISCONNECTED));
		System.out.println("rbsmnode start");
	}
	
	@Override
	public void setSerialPort(SerialPort sp) 
	{
		super.setSerialPort(sp);
		statePub_enc.publish(new StateMessage(SensorState.ON));
		statePub_pot.publish(new StateMessage(SensorState.ON));
	}
	
	private void estimateVelocity(int dataWord) 
	{
		Date currTime = new Date();
		double accDist = ((double)(encTicks)) * M_PER_REV / TICKS_PER_REV;
		double instVelocity = (accDist - accDistLast) * 1000 / (currTime.getTime() - timeLast.getTime());
		double instAccel = (instVelocity - instVelocityLast) * 1000 / (currTime.getTime() - timeLast.getTime());
		accDistLast = accDist;
		instVelocityLast = instVelocity;
		timeLast = currTime;	
		
		messagePub_enc.publish(new EncoderMeasurement(currTime, dataWord, accDist, instVelocity, instAccel));
	}
	
	@Override
	public boolean matchDataSample(byte[] sample) 
	{
		// Peel what ever is not a message
		
		// Check that whatever we give it is a message
		
		return true;
	}

	@Override
	public int matchDataMinSize() 
	{
		return 2*RBSerial.MSG_LEN;
	}

	@Override
	//must be the same as the baud rate of the arduino 
	public int baudRate() 
	{
		return 115200;
	}

	@Override
	public int peel(byte[] buffer, int start, int bytes_available) 
	{
		System.out.println("Stuff is going on\n\n\n\n");
		// The Encoder sends 3 types of messages
		//  - Encoder ticks since last message (keep)
		//  - Number of ticks since last reset
		//  - Timestamp since reset
		RBPair rbp = RBSerial.peel(buffer, start, bytes_available);
		switch(rbp.getNumberOfBytesRead()) 
		{
			case 0: return 0;
			case 1: return 1;
			case 6: break;
			default: 
			{
				System.out.println("HOW DID NOT A SIX GET HERE");
				//TODO add error
			}
		
		}
		
		RBSerialMessage message = rbp.getMessage();
		if(message.getHeaderByte() == RBSerialMessage.ENC_TICK_SINCE_RESET) 
		{
			// This is a delta-distance! Do a thing!
			encTicks = message.getDataWord() & 0xFFF;
			estimateVelocity(message.getDataWord());
			System.out.println(encTicks);
		}
		
		else if(message.getHeaderByte() == RBSerialMessage.RBSM_MID_MEGA_STEER_FEEDBACK) 
		{
			// This is a delta-distance! Do a thing!
			potValue = message.getDataWord();
			System.out.println(potValue);
			messagePub_pot.publish(new SteeringMeasurement(-(potValue + OFFSET)/ARD_TO_DEG));
		}

		else if (message.getHeaderByte() == RBSerialMessage.FP_HASH)
		{
			System.out.println(message.getDataWord());
		}
		
		
		return 6;
	}

}


