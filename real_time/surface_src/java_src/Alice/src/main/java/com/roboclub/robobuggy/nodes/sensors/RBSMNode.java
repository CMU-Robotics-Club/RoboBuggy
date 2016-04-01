package com.roboclub.robobuggy.nodes.sensors;

import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.messages.AutonBrakeStateMessage;
import com.roboclub.robobuggy.messages.AutonStateMessage;
import com.roboclub.robobuggy.messages.BatteryLevelMessage;
import com.roboclub.robobuggy.messages.BrakeControlMessage;
import com.roboclub.robobuggy.messages.BrakeStateMessage;
import com.roboclub.robobuggy.messages.DeviceIDMessage;
import com.roboclub.robobuggy.messages.DriveControlMessage;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.EncoderTimeMessage;
import com.roboclub.robobuggy.messages.FingerPrintMessage;
import com.roboclub.robobuggy.messages.StateMessage;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.messages.TeleopBrakeStateMessage;
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
 *
 * CHANGELOG: Converted to use the decorator pattern implementation of
 * {@link Node}s for buggy
 * 
 * DESCRIPTION: node for talking to the the low level controller via rbsm
 */

public  class  RBSMNode extends SerialNode {
	
	private static final int BAUD_RATE = 76800;
	
	private static final double TICKS_PER_REV = 7.0;
	// Measured as 2 feet. Though could be made more precise. 
	private static final double M_PER_REV = 0.61;
	
	/** Steering Angle Conversion Rate */
	private static final double ARD_TO_DEG = 1;
	/** Steering Angle offset?? */
	private static final double OFFSET = 0;

	// accumulated
	private int encTicks = 0;
	private double potValue = -1.0;


	// last state
	private double accDistLast = 0.0;
	private double instVelocityLast = 0.0;
	private Date timeLast = new Date();
	
	private Publisher messagePubEnc;
	private Publisher messagePubPot;
	private Publisher messagePubControllerSteering;
	private Publisher messagePubBat;
	
	private Publisher messagePubDeviceID;
	private Publisher messagePubAutonState;
	private Publisher messagePubBrakeState;
	
	private Publisher messagePubAutonBrakeState;
	private Publisher messagePubTeleopBrakeState;
	
	private Publisher statePubEnc;
	private Publisher statePubPot;
	
	private Publisher messagePubEncTime;
	
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
		messagePubFp = new Publisher(NodeChannel.FP_HASH.getMsgPath());
		
		messagePubBat = new Publisher(NodeChannel.BATTERY.getMsgPath());
		
		messagePubEncTime = new Publisher(NodeChannel.ENCODERTIME.getMsgPath());
		
		messagePubDeviceID = new Publisher(NodeChannel.DEVICE_ID.getMsgPath());
		messagePubAutonState = new Publisher(NodeChannel.AUTON_STATE.getMsgPath());
		messagePubBrakeState = new Publisher(NodeChannel.BRAKE_STATE.getMsgPath());
		
		messagePubAutonBrakeState = new Publisher(NodeChannel.AUTON_BRAKE_STATE.getMsgPath());
		messagePubTeleopBrakeState = new Publisher(NodeChannel.TELEOP_BRAKE_STATE.getMsgPath());

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
		double accDist = (((double)(encTicks)) * M_PER_REV / TICKS_PER_REV)/.49;
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
				new RobobuggyLogicNotification("Peel did not evaluate to 0,1, or 6 in RBSMNode", RobobuggyMessageLevel.EXCEPTION);
				break;
		}
		
		RBSerialMessage message = rbp.getMessage();
		int headerNumber = message.getHeaderNumber();
	
		//TODO: The following statements should be reformatted because:
		// 1: Calling getHeaderByte so many times is inefficient
		// 2: Header values don't change so this can be made into a switch 
		// 3: Header values are subject to change based on what's in rbsm_headers.txt
		if (headerNumber == RBSerialMessage.getHeaderByte("RBSM_MID_ENC_TICKS_RESET")) {
			// This is a delta-distance! Do a thing!
			encTicks = message.getDataWord() & 0xFFFF;
			messagePubEnc.publish(estimateVelocity(message.getDataWord()));
		}
		else if (headerNumber == RBSerialMessage.getHeaderByte("RBSM_MID_ERROR")) {
			new RobobuggyLogicNotification("RBSM_MID_ERROR:"+message.getDataWord(), RobobuggyMessageLevel.EXCEPTION);
		}
		else if (headerNumber == RBSerialMessage.getHeaderByte("RBSM_MID_ENC_RESET_CONFIRM")) {
			new RobobuggyLogicNotification("Encoder Reset Confirmed by Zoe", RobobuggyMessageLevel.NOTE);
		}
		else if (headerNumber == RBSerialMessage.getHeaderByte("RBSM_MID_ENC_TIMESTAMP")){
			messagePubEncTime.publish(new EncoderTimeMessage(message.getDataWord()));
		}
		else if (headerNumber == RBSerialMessage.getHeaderByte("RBSM_MID_MEGA_STEER_FEEDBACK")) {
			potValue = message.getDataWord();
			messagePubPot.publish(new SteeringMeasurement(-(potValue + OFFSET) / ARD_TO_DEG));
		}
		else if (headerNumber == RBSerialMessage.getHeaderByte("RBSM_MID_MEGA_STEER_ANGLE")) {
			messagePubControllerSteering.publish(new SteeringMeasurement(message.getDataWord()));
		}
		else if (headerNumber == RBSerialMessage.getHeaderByte("RBSM_MID_COMP_HASH")) {
			messagePubFp.publish(new FingerPrintMessage(message.getDataWord()));
		}
		else if (headerNumber == RBSerialMessage.getHeaderByte("RBSM_MID_MEGA_BATTERY_LEVEL")){
			//TODO: Display the battery level in the GUI
			messagePubBat.publish(new BatteryLevelMessage(message.getDataWord()));
		}
		else if (headerNumber == RBSerialMessage.getHeaderByte("DEVICE_ID")){
			messagePubDeviceID.publish(new DeviceIDMessage(message.getDataWord()));
		}
		else if (headerNumber == RBSerialMessage.getHeaderByte("RBSM_MID_MEGA_BRAKE_STATE")){
			boolean brakesDown = false;

			if (message.getDataWord() == 1) {
				brakesDown = true;
			}

			messagePubBrakeState.publish(new BrakeStateMessage(brakesDown));
		}
		else if (headerNumber == RBSerialMessage.getHeaderByte("RBSM_MID_MEGA_AUTON_STATE")){
			messagePubAutonState.publish(new AutonStateMessage(message.getDataWord()));
		}
		else if (headerNumber == RBSerialMessage.getHeaderByte("RBSM_MID_MEGA_TELEOP_BRAKE_COMMAND")){
			messagePubTeleopBrakeState.publish(new TeleopBrakeStateMessage(message.getDataWord()));
		}
		else if (headerNumber == RBSerialMessage.getHeaderByte("RBSM_MID_MEGA_AUTON_BRAKE_COMMAND")){
			messagePubAutonBrakeState.publish(new AutonBrakeStateMessage(message.getDataWord()));			
		}
		else {
				new RobobuggyLogicNotification("Invalid RBSM message header: " + headerNumber+ 
						" Message:"+message.getDataWord(), RobobuggyMessageLevel.NOTE);
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
		private int commandedAngle = 0;
		private boolean commandedBrakeEngaged = true;
		
		/**
		 * Create a new {@link RBSMPeriodicNode} object
		 * @param period of the periodic behavior
		 * @param channel of the RSBM node
		 */
		RBSMPeriodicNode(NodeChannel channel, int period) {
			super(new BuggyBaseNode(channel), period);
		}

		/**
		 * Used to send the commanded angle and brake state to the Arduino.
		 * {@inheritDoc}
		 */
		@Override
		protected void update() {
			int outputAngle = commandedAngle;//allows for commandedAngle to be read only in this function
			if(outputAngle > 1000) {
				outputAngle = 1000;
			}
			else if (outputAngle < -1000) {
				outputAngle = -1000;
			}
			
			RBSMSteeringMessage msgSteer = new RBSMSteeringMessage(outputAngle);
			send(msgSteer.getMessageBytes());
			RBSMBrakeMessage msgBrake = new RBSMBrakeMessage(commandedBrakeEngaged);
			send(msgBrake.getMessageBytes());
		}

		/**{@inheritDoc}*/
		@Override
		protected boolean startDecoratorNode() {
			//Initialize subscribers to commanded angle and brakes state
			new Subscriber(NodeChannel.DRIVE_CTRL.getMsgPath(),
					new MessageListener() {
				@Override
				public void actionPerformed(String topicName, Message m) {
					commandedAngle = ((DriveControlMessage)m).getAngleInt();
				}
			});
			new Subscriber(NodeChannel.BRAKE_CTRL.getMsgPath(),
					new MessageListener() {
				@Override
				public void actionPerformed(String topicName, Message m) {
					commandedBrakeEngaged = ((BrakeControlMessage)m).isBrakeEngaged();
				}
			});
			new Subscriber(NodeChannel.ENCODER_RESET.getMsgPath(), new MessageListener() {
				@Override
				public void actionPerformed(String topicName, Message m) {
					byte[] message = new byte[6];
					message[0] = (byte)RBSerialMessage.getHeaderByte("RBSM_MID_ENC_RESET_REQUEST"); //Reset request header
					message[1] = 0;
					message[2] = 0;
					message[3] = 0;
					message[4] = 0;
					message[5] = (byte)RBSerialMessage.getHeaderByte("FOOTER");
					send(message);
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
