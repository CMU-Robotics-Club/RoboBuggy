package com.roboclub.robobuggy.nodes.sensors;

import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.messages.BrakeControlMessage;
import com.roboclub.robobuggy.messages.DriveControlMessage;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.FingerPrintMessage;
import com.roboclub.robobuggy.messages.StateMessage;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyBaseNode;
import com.roboclub.robobuggy.nodes.baseNodes.NodeState;
import com.roboclub.robobuggy.nodes.baseNodes.PeriodicNode;
import com.roboclub.robobuggy.nodes.baseNodes.SerialNode;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.Subscriber;
import com.roboclub.robobuggy.serial.RBPair;
import com.roboclub.robobuggy.serial.RBSerial;
import com.roboclub.robobuggy.serial.RBSerialMessage;

import java.util.Date;

/**
 * {@link SerialNode} for reading and sending RBSM data
 * @author Trevor Decker
 * @author Matt Sebek
 * @author Kevin Brennan
 * @author Zachary Dawson
 *
 * CHANGELOG: Converted to use the decorator pattern implementation of
 * {@link Node}s for buggy
 * 
 * DESCRIPTION: node for talking to the the low level controller via rbsm
 */

public class RBSMNode extends SerialNode {
	
	private static final int BAUD_RATE = 76800;
	
	private static final double TICKS_PER_REV = 7.0;
	// Measured as 2 feet. Though could be made more precise. 
	private static final double M_PER_REV = 0.61;
	
	/** Steering Angle Conversion Rate */
	private static final int ARD_TO_DEG = 100;
	/** Steering Angle offset?? */
	private static final int OFFSET = -200;

	// accumulated
	private int encTicks = 0;
	private int potValue = -1;
	private int steeringAngle = 0;
	
	// last state
	private double accDistLast = 0.0;
	private double instVelocityLast = 0.0;
	private Date timeLast = new Date();
	
	private Publisher messagePubEnc;
	private Publisher messagePubPot;
	private Publisher messagePubControllerSteering;
//	private Publisher brakePub; future expansion
	
	private Publisher statePubEnc;
	private Publisher statePubPot;
	
	private Publisher messagePubFp; //Fingerprint for low level hash
	
	/**
	 * Construct a new RBSMNode object
	 * @param sensorEnc channel the encoder is on
	 * @param sensorPot channel the potentiometer is on
	 * @param portName name of the serial port used to read from the Arduino
	 * @param period the period at which control messages are sent to the Arduino
	 */
	public RBSMNode(NodeChannel sensorEnc, NodeChannel sensorPot,
			String portName, int period) {
		super(null, "RBSM", portName, BAUD_RATE);
		this.setBaseNode(new RBSMPeriodicNode(sensorEnc, period));
		//messagePubs forward exact information received from arduino
		messagePubEnc = new Publisher(sensorEnc.getMsgPath());
		messagePubPot = new Publisher(sensorPot.getMsgPath());
		messagePubControllerSteering = new Publisher(NodeChannel.STEERING_COMMANDED.getMsgPath());
//		brakePub = new Publisher(NodeChannel.BRAKE.getMsgPath()); //todo future expansion 
		messagePubFp = new Publisher(NodeChannel.FP_HASH.getMsgPath());

		//statePub forwards this node's estimate of the state
		//(i.e. after filtering bad data)
		statePubEnc = new Publisher(sensorEnc.getStatePath());
		statePubPot = new Publisher(sensorPot.getStatePath());

		//Initialize publishers to indicate the sensor is disconnected
		statePubEnc.publish(new StateMessage(NodeState.DISCONNECTED));
		statePubPot.publish(new StateMessage(NodeState.DISCONNECTED));
	}
	
	/**
	 * Calculates an estimate of the velocity from an encoder measurement
	 * @param dataWord raw data bits received from the RBSM message
	 * @return an EncoderMeasurement object representing the velocity
	 */
	private EncoderMeasurement estimateVelocity(int dataWord) 
	{
		Date currTime = new Date();
		double accDist = ((double)(encTicks)) * M_PER_REV / TICKS_PER_REV;
		double instVelocity = (accDist - accDistLast) * 1000 / (currTime.getTime() - timeLast.getTime());
		double instAccel = (instVelocity - instVelocityLast) * 1000 / (currTime.getTime() - timeLast.getTime());
		accDistLast = accDist;
		instVelocityLast = instVelocity;
		timeLast = currTime;
		
		return new EncoderMeasurement(currTime, dataWord, accDist, instVelocity, instAccel);
	}
	
	/**{@inheritDoc}*/
	@Override
	public boolean matchDataSample(byte[] sample) 
	{
		// Peel what ever is not a message
		
		// Check that whatever we give it is a message
		
		return true;
	}

	/**{@inheritDoc}*/
	@Override
	public int matchDataMinSize() 
	{
		return 2*RBSerial.MSG_LEN;
	}

	/**{@inheritDoc}*/
	@Override
	//must be the same as the baud rate of the arduino 
	public int getBaudRate() 
	{
		return BAUD_RATE;
	}

	
	/**{@inheritDoc}*/
	@Override
	public int peel(byte[] buffer, int start, int bytesAvailable) 
	{
		// The Encoder sends 3 types of messages
		//  - Encoder ticks since last message (keep)
		//  - Number of ticks since last reset
		//  - Timestamp since reset
		RBPair rbp = RBSerial.peel(buffer, start, bytesAvailable);
		switch(rbp.getNumberOfBytesRead()) 
		{
			case 0:
				return 0;
			case 1: 
				return 1;
			case 6: 
				break;
			default: 
				System.out.println("HOW DID NOT A SIX GET HERE");
				//TODO add error
				break;
		}
		
		RBSerialMessage message = rbp.getMessage();
		byte headerByte = message.getHeaderByte();
		switch (headerByte)
		{
			case RBSerialMessage.ENC_TICK_SINCE_RESET:
				// This is a delta-distance! Do a thing!
				encTicks = message.getDataWord() & 0xFFFF;
				messagePubEnc.publish(estimateVelocity(message.getDataWord()));
				break;
			case RBSerialMessage.RBSM_MID_MEGA_STEER_FEEDBACK:
				// This is a delta-distance! Do a thing!
				potValue = message.getDataWord();
				System.out.println(potValue);
				messagePubPot.publish(new SteeringMeasurement(-(potValue + OFFSET)/ARD_TO_DEG));
				break;
			case RBSerialMessage.RBSM_MID_MEGA_STEER_ANGLE:
				steeringAngle = message.getDataWord();
				messagePubControllerSteering.publish(new SteeringMeasurement(steeringAngle));
				break;
			case RBSerialMessage.FP_HASH:
				System.out.println(message.getDataWord());
				messagePubFp.publish(new FingerPrintMessage(message.getDataWord()));
				break;
			default: //Unhandled or invalid RBSM message header was received.
				new RobobuggyLogicNotification("Invalid RBSM message header: " + headerByte, RobobuggyMessageLevel.NOTE);
				break;
		}
		
		//Feed the watchdog
		setNodeState(NodeState.ON);
		
		return 6;
	}

	/**
	 * Private class used to handle the periodic portions of the
	 * {@link RBSMNode}. Specifically, this will transmit the commanded
	 * steering and brake values periodically.
	 * 
	 * @author Zachary Dawson
	 */
	private final class RBSMPeriodicNode extends PeriodicNode {
		
		//Stored commanded values
		private short commandedAngle = 0;
		private boolean commandedBrakeEngaged = true;
		
		/**
		 * Create a new {@link RBSMPeriodicNode} object
		 * @param period of the periodic behavior
		 * @param channel of the RSBM node
		 */
		public RBSMPeriodicNode(NodeChannel channel, int period) {
			super(new BuggyBaseNode(channel), period);
		}

		/**
		 * Used to send the commanded angle and brake state to the Arduino.
		 * {@inheritDoc}
		 */
		@Override
		protected void update() {
			commandedAngle = (short) ((commandedAngle + 1) % 256);
			System.out.println("sending command "+commandedAngle);
			RBSMessage msg = new RBSMessage(commandedAngle, commandedBrakeEngaged);
			send(msg.getMessageBytes());
		}

		/**{@inheritDoc}*/
		@Override
		protected boolean startDecoratorNode() {
			//Initialize subscribers to commanded angle and brakes state
			new Subscriber(NodeChannel.DRIVE_CTRL.getMsgPath(),
					new MessageListener() {
				@Override
				public void actionPerformed(String topicName, Message m) {
					commandedAngle = ((DriveControlMessage)m).getAngleShort();
				}
			});
			new Subscriber(NodeChannel.BRAKE_CTRL.getMsgPath(),
					new MessageListener() {
				@Override
				public void actionPerformed(String topicName, Message m) {
					commandedBrakeEngaged = ((BrakeControlMessage)m).isBrakeEngaged();
				}
			});
			return true;
		}

		/**{@inheritDoc}*/
		@Override
		protected boolean shutdownDecoratorNode() {
			return true;
		}	
	}
}


