package com.roboclub.robobuggy.sensors;

import com.roboclub.robobuggy.main.Robot;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.serial.Arduino;
import com.roboclub.robobuggy.serial.SerialEvent;
import com.roboclub.robobuggy.serial.SerialListener;

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
	public static final char STEERING = (char)0x05;
	public static final char BRAKE = (char)0x06;
	
	//private Publisher encoderPub = new Publisher("/sensor/drive");
	
	public DriveActuator() {
		super("Steering", 1);
		
		if (port != null && port.isConnected()) super.addListener(new DriveListener());
	}
	
	/* Methods for Serial Communication with Arduino */
	public void writeAngle(int angle) {
		if (angle >= 0 && angle <= 180) {
			if(isConnected()) {
				byte[] msg = {0x05, 0, 0, 0, (byte)angle,'\n'};
				msg[2] = (byte)angle;
				this.port.serialWrite(msg);
			}
		}
	}
	
	public void writeBrake(int angle) {
		if (angle >= 0 && angle <= 180) {
			if (isConnected()) {
				byte[] msg = {0x06, 0, 0, 0, (byte)angle,'\n'};
				msg[2] = (byte)angle;
				this.port.serialWrite(msg);
			}
		}
	}
	
	/* Methods for reading from Serial */
	public static boolean validId(char value) {
		switch (value) {
			case STEERING:
			case BRAKE:
			case Arduino.ERROR:
				return true;
			default:
				return false;
		}
	}
	
	public class DriveListener implements SerialListener {
		@Override
		public void onEvent(SerialEvent event) {
			char[] tmp = event.getBuffer();
			if (event.getLength() != MSG_LEN) return;
			
			switch (tmp[0]) {
			case STEERING:
				Robot.UpdateSteering(tmp[4]);
				break;
			case BRAKE:
				Robot.UpdateBrake(tmp[4]);
				break;
			case ERROR:
				Robot.UpdateError(parseInt(tmp[1], tmp[2], tmp[3], tmp[4]));
				break;
			default:
				return;
			}
		}
	}
}
