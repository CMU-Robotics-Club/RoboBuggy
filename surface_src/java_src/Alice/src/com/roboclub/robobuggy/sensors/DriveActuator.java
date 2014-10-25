package com.roboclub.robobuggy.sensors;

import com.roboclub.robobuggy.main.Robot;
import com.roboclub.robobuggy.messages.BrakeCommand;
import com.roboclub.robobuggy.messages.WheelAngleCommand;
import com.roboclub.robobuggy.serial.Arduino;

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

public class DriveActuator extends Arduino {
	
   public int steeringAngle;
	
	public DriveActuator() {
		super("Steering", "/sensor/drive");
		thisSensorType = SensorType.GPS;
	}
	
	/* Methods for Serial Communication with Arduino */
	public void writeAngle(int angle) {
		if (angle >= 0 && angle <= 180) {
			if(isConnected()) {
				byte[] msg = {0x05, 0, 0, 0, (byte)angle,'\n'};
				msg[2] = (byte)angle;
				super.serialWrite(msg);
			}
		}
	}
	
	public void writeBrake(int angle) {
		if (angle >= 0 && angle <= 180) {
			if (isConnected()) {
				byte[] msg = {0x06, 0, 0, 0, (byte)angle,'\n'};
				msg[2] = (byte)angle;
				super.serialWrite(msg);
			}
		}
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
		currentState = SensorState.ON;
		lastUpdateTime = System.currentTimeMillis();
		
		try {
			switch (inputBuffer[0]) {
			case STEERING:
				Robot.UpdateSteering(inputBuffer[4]);
				publisher.publish(new WheelAngleCommand((byte) inputBuffer[4]));
				System.out.println("Steering Angle:" + Integer.toHexString((int)inputBuffer[4]));
				steeringAngle = (int)inputBuffer[4];
				break;
			case BRAKE:
				Robot.UpdateBrake(inputBuffer[4]);
				publisher.publish(new BrakeCommand(true));
				break;
			case ERROR:
				Robot.UpdateError(parseInt(inputBuffer[1], inputBuffer[2],
						inputBuffer[3], inputBuffer[4]));
				break;
			}
		} catch (Exception e) {
			System.out.println("Drive Exception on port: " + this.getName());
		}
	}
}