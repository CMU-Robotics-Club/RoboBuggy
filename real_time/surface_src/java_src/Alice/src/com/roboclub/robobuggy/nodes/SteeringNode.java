package com.roboclub.robobuggy.nodes;

import com.roboclub.robobuggy.messages.BrakeMessage;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.serial.RBPair;
import com.roboclub.robobuggy.serial.RBSerial;
import com.roboclub.robobuggy.serial.RBSerialMessage;
import com.roboclub.robobuggy.serial.SerialNode;

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

public class SteeringNode extends SerialNode implements Node {
	public Publisher brakePub;
	public Publisher steeringPub;
	public Publisher statePub;
	
	public SteeringNode(SensorChannel sensor) {
		super("Steering");
		
		brakePub = new Publisher(SensorChannel.BRAKE.getMsgPath());
		steeringPub = new Publisher(SensorChannel.STEERING.getMsgPath());
		statePub = new Publisher(sensor.getStatePath());	
		
		// Subscriber for Steering commands
		/*new Subscriber(ActuatorChannel.STEERING.getMsgPath(),
				new MessageListener() {
					@Override
					public void actionPerformed(String topicName, Message m) {
						if (currState == SensorState.ON) {
							writeAngle(((WheelAngleCommand) m).angle);
						}
					}
		});
		
		//Subscriber for Brake Commands
		new Subscriber(ActuatorChannel.BRAKE.getMsgPath(),
				new MessageListener() {
					@Override
					public void actionPerformed(String topicName, Message m) {
						if (currState == SensorState.ON) {
							writeBrake(((BrakeCommand)m).down);
						}
					}
		});*/
	}

	@Override
	public boolean matchDataSample(byte[] sample) {
		// TODO actually use this
		return true;
	}

	@Override
	public int matchDataMinSize() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public int baudRate() {
		return 9600;
	}

	@Override
	public int peel(byte[] buffer, int start, int bytes_available) {
		// The drive controller sends 2 types of messages
		//	- Steering angle
		//	- Brake deployment or release
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
		switch (message.getHeaderByte()) {
			case RBSerialMessage.BRAKE: {
				brakePub.publish(new BrakeMessage(message.getDataWord()));
			}
			case RBSerialMessage.STEERING: {
				steeringPub.publish(new SteeringMeasurement(message.getDataWord()));
			}
		}
		
		
		return 6;
	}
}
