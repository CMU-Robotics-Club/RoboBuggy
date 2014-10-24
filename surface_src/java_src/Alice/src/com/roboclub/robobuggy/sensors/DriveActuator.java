package com.roboclub.robobuggy.sensors;

import gnu.io.SerialPortEvent;
import com.roboclub.robobuggy.main.Robot;
import com.roboclub.robobuggy.messages.BrakeCommand;
import com.roboclub.robobuggy.messages.WheelAngleCommand;
import com.roboclub.robobuggy.serial.Arduino;

public class DriveActuator extends Arduino {
	
	public DriveActuator() {
		super("Steering", "/sensor/drive");
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
			case Arduino.ERROR:
				return true;
			default:
				return false;
		}
	}
	
	@Override
	public void serialEvent(SerialPortEvent event) {
		switch (event.getEventType()) {
		case SerialPortEvent.DATA_AVAILABLE:
			try {
				char data = (char)input.read();
				
				switch (state) {
				case 0:
					if (validId(data)) {
						inputBuffer[index++] = data;
						state++;
					}
					
					break;
				case 1:
					inputBuffer[index++] = data;
					
					if (index == MSG_LEN) {
						if (data == '\n') publish();
						index = 0;
						state = 0;
					}
					break;
				}
			} catch (Exception e) {
				System.out.println("Encoder Exception in port: " + this.getName());
			}
		}
	}

	@Override
	public void publish() {
		currentState = SensorState.ON;
		lastUpdateTime = System.currentTimeMillis();
		
		switch (inputBuffer[0]) {
		case STEERING:
			Robot.UpdateSteering(inputBuffer[4]);
			publisher.publish(new WheelAngleCommand((byte) inputBuffer[4]));
			break;
		case BRAKE:
			Robot.UpdateBrake(inputBuffer[4]);
			publisher.publish(new BrakeCommand(true));
			break;
		case ERROR:
			Robot.UpdateError(parseInt(inputBuffer[1], inputBuffer[2],
					inputBuffer[3], inputBuffer[4]));
			break;
		default:
			return;
		}
	}
}
