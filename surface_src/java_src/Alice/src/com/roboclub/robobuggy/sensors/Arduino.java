package com.roboclub.robobuggy.sensors;

import java.nio.ByteBuffer;

import com.roboclub.robobuggy.main.Robot;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.serial.SerialEvent;
import com.roboclub.robobuggy.serial.SerialListener;

public class Arduino extends Sensor {
	public static final int MSG_LEN = 8;
	public static final char ENC_TIME = (char)0x00;
	public static final char ENC_RESET = (char)0x01;
	public static final char ENC_TICK = (char)0x02;
	public static final char STEERING = (char)0x05;
	public static final char BRAKE = (char)0x06;
	public static final char ERROR = (char)0xFF;
	private static final int BAUDRATE = 9600;
	
	private static final long TICKS_PER_REV = 5;
	private static final double M_PER_REV = 5.0;
	
	private int encReset;
	private int encTickLast;
	private int encTime;

	// Set up publishers
	// TODO refactor into string PublishPath argument to Arduino()
	private Publisher encoderPub = new Publisher("/sensor/encoder");
	
	public Arduino() {
		super("Arduino", BAUDRATE, null);
		
		System.out.println("Initializing Arudino");
		if (port != null && port.isConnected()) super.addListener(new ArduinoListener());
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
	
	private void calcDist() {
		double dist = ((double)(encTickLast - encReset)/TICKS_PER_REV) / M_PER_REV;
		double velocity = dist / encTime;
		
		Robot.UpdateEnc(dist, velocity);
		//encoderPub.publish(new EncoderMeasurement(dist, velocity));
	}
	
	public static boolean validId(char value) {
		switch (value) {
			case Arduino.ENC_TIME:
			case Arduino.ENC_RESET:
			case Arduino.ENC_TICK:
			case Arduino.STEERING:
			case Arduino.BRAKE:
			case Arduino.ERROR:
				return true;
			default:
				return false;
		}
	}
	
	private class ArduinoListener implements SerialListener {
		@Override
		public void onEvent(SerialEvent event) {
			char[] tmp = event.getBuffer();
			if (event.getLength() != MSG_LEN) return;
			byte[] msg = {(byte)tmp[1], (byte)tmp[2], (byte)tmp[3], (byte)tmp[4]};
			ByteBuffer wrapper = ByteBuffer.wrap(msg);
			
			switch (tmp[0]) {
			case ENC_TIME:
				encTime = wrapper.getInt();
			case ENC_RESET:
				encReset = wrapper.getInt();
			case ENC_TICK:
				encTickLast = wrapper.getInt();
				Robot.UpdateEnc(encTime, encReset, encTickLast);
			case STEERING:
				Robot.UpdateSteering(tmp[4]);
			case BRAKE:
				Robot.UpdateBrake(tmp[4]);
			case ERROR:
				Robot.UpdateError(wrapper.getInt());
			default:
				return;
			}
		}
	}
}
