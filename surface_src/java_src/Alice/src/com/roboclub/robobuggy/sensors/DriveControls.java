package com.roboclub.robobuggy.sensors;

import com.roboclub.robobuggy.messages.BrakeCommand;
import com.roboclub.robobuggy.messages.StateMessage;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.messages.SteeringCommand;
import com.roboclub.robobuggy.ros.CommandChannel;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.ros.Subscriber;

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

public class DriveControls extends Arduino {
	
   public int steeringAngle;
	
	public DriveControls(SensorChannel sensor) {
		super(sensor, "Steering");
		sensorType = SensorType.GPS;
		
		// Subscriber for Steering commands
		new Subscriber(CommandChannel.STEERING.getMsgPath(),
				new MessageListener() {
					@Override
					public void actionPerformed(String topicName, Message m) {
						if (currState == SensorState.ON) {
							writeAngle(((SteeringCommand)m).angle);
						}
					}
		});
		
		//Subscriber for Brake Commands
		new Subscriber(CommandChannel.BRAKE.getMsgPath(),
				new MessageListener() {
					@Override
					public void actionPerformed(String topicName, Message m) {
						if (currState == SensorState.ON) {
							writeBrake(((BrakeCommand)m).down);
						}
					}
		});
	}
	
	/* Methods for Serial Communication with Arduino */
	public void writeAngle(int angle) {
		if (angle >= 0 && angle <= 180) {
			if(isConnected()) {
				byte[] msg = {
						(byte)((angle >> 0x18) & 0xFF),
						(byte)((angle >> 0x10) & 0xFF),
						(byte)((angle >> 0x08) & 0xFF),
						(byte)(angle & 0xFF),'\n'};
				super.serialWrite(msg);
			}
		}
	}
	
	public void writeBrake(boolean value) {
		// TODO writing brake command
	}
	
	/* Methods for reading from Serial */
	@Override
	protected boolean validId(char value) {
		switch (value) {
			case STEERING:
			case BRAKE:
			case ERROR:
			case MSG_ID:
				return true;
			default:
				return false;
		}
	}

	@Override
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
	}
}
