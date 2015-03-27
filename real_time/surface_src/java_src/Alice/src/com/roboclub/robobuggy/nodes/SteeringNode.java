package com.roboclub.robobuggy.nodes;

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
		
		//brakePub = new Publisher();
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
	
	/* Methods for Serial Communication with Arduino */
	/*public void writeAngle(float angle) {
		if (angle >= 0 && angle <= 180) {
			if(isConnected()) {
				/*byte[] msg = {
						(byte)((angle >> 0x18) & 0xFF),
						(byte)((angle >> 0x10) & 0xFF),
						(byte)((angle >> 0x08) & 0xFF),
						(byte)(angle & 0xFF),'\n'};
				System.out.println("MATT BROKE THIS BECAUSE INTERFACE WITH LOW LEVEL CHANGED");
				//super.serialWrite(null);
			}
		}
	}*/
	
	/*@Override
	public void publish() {
		lastUpdateTime = System.currentTimeMillis();
		int value = parseInt(inputBuffer[1], inputBuffer[2],
				inputBuffer[3], inputBuffer[4]);
		try {
			switch (inputBuffer[0]) {
			case STEERING:
				msgPub.publish(new SteeringMeasurement(value));
				break;
			case BRAKE:
				// TODO handle brake messages
				break;
			case ERROR:
				// TODO handle error messages
				break;
			}
		} catch (Exception e) {
			System.out.println("Drive Exception on port: " + this.getName());
			if (currState != SensorState.FAULT) {
				currState = SensorState.FAULT;
				statePub.publish(new StateMessage(this.currState));
				return;
			}
		}
		
		if (currState != SensorState.ON) {
			currState = SensorState.ON;
			statePub.publish(new StateMessage(this.currState));
		}
	}*/

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
		RBPair rbp = RBSerial.peel(buffer, start, bytes_available);
		int bytes_read = rbp.getNumberOfBytesRead();
		RBSerialMessage message = rbp.getMessage();
	
		byte b = message.getHeaderByte();
		if(b == RBSerialMessage.BRAKE) {
			
		} else if (b == RBSerialMessage.STEERING){
			
			
		} else if (b == RBSerialMessage.DEVICE_ID) {
			// Do nothing really
			
		}
		
		return bytes_read;
	}
}
