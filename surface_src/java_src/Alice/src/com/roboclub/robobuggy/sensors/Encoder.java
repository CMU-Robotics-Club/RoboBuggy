package com.roboclub.robobuggy.sensors;

import gnu.io.SerialPortEvent;
import com.roboclub.robobuggy.main.Robot;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.serial.Arduino;

public class Encoder extends Arduino {
	private static final long TICKS_PER_REV = 5;
	private static final double M_PER_REV = 5.0;
	
	private int encReset;
	private int encTicks;
	private int encTime;
	private double dist;
	private double velocity;
	
	public Encoder() {
		super("Encoder", "/sensor/encoder");
	}
	
	//is run after the encoder gets new data to update internal sate variables 
	public void updated()
	{
		lastUpdateTime = System.currentTimeMillis();
		currentState = SensorState.ON; //TODO fix this 
		
		dist = ((double)(encTicks - encReset)/TICKS_PER_REV) / M_PER_REV;
		velocity = dist / encTime;	
		
		publisher.publish(new EncoderMeasurement(dist, velocity));
	}
	
	/* Methods for reading from Serial */
	@Override
	public boolean validId(char value) {
		switch (value) {
			case ENC_TIME:
			case ENC_RESET:
			case ENC_TICK:
			case ERROR:
				return true;
			default:
				return false;
		}
	}
	
	@Override
	public void publish() {
		currentState = SensorState.ON;
		lastUpdateTime = System.currentTimeMillis();
		
		System.out.println((int)inputBuffer[0] + " " + (int)inputBuffer[1] + 
				" " + (int)inputBuffer[2] + " " + (int)inputBuffer[3]);
		switch (inputBuffer[0]) {
		case ENC_TIME:
			encTime = parseInt(inputBuffer[1], inputBuffer[2], inputBuffer[3], inputBuffer[4]);
			break;
		case ENC_RESET:
			encReset = parseInt(inputBuffer[1], inputBuffer[2], inputBuffer[3], inputBuffer[4]);
			break;
		case ENC_TICK:
			System.out.println("Time: " + encTime + " Reset: " + encReset + " Ticks: " + encTicks);
			encTicks = parseInt(inputBuffer[1], inputBuffer[2], inputBuffer[3], inputBuffer[4]);
			Robot.UpdateEnc(encTime, encReset, encTicks);
			break;
		case ERROR:
			Robot.UpdateError(parseInt(inputBuffer[1], inputBuffer[2], inputBuffer[3], inputBuffer[4]));
			break;
		default:
			System.out.println("Invalid Encoder Message!");
		}
	}

	/*%*****		Serial Methods			*****%*/
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
}
