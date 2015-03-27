package com.roboclub.robobuggy.nodes;

import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.messages.StateMessage;
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.sensors.SensorState;
import com.roboclub.robobuggy.serial2.SerialNode;

/**
 * @author Matt Sebek 
 * @author Kevin Brennan
 *
 * DESCRIPTION: TODO
 */

public class ImuNode extends SerialNode implements Node {
	/** Baud rate for serial port */
	private static final int BAUDRATE = 57600;
	// how long the system should wait until a sensor switches to Disconnected
	private static final long SENSOR_TIME_OUT = 5000;

	// Used for state estimation
	/*private static final double GYRO_PER = 0.9;
	private static final double ACCEL_PER = 1 - GYRO_PER;
	public double angle;
	*/

	public Publisher msgPub;
	public Publisher statePub;
	
	public ImuNode(SensorChannel sensor) {
		super("IMU");
		msgPub = new Publisher(sensor.getMsgPath());
		statePub = new Publisher(sensor.getStatePath());
	
		// TODO state stuff
		statePub.publish(new StateMessage(SensorState.DISCONNECTED));
	}

	@Override
	public void setSerialPort(gnu.io.SerialPort sp) {
		super.setSerialPort(sp);
		statePub.publish(new StateMessage(SensorState.ON));
	};
	
	/*
	@SuppressWarnings("unused")
	private void estimateOrientation(double aX, double aY, double aZ, 
			double rX, double rY, double rZ, double mX, double mY, double mZ) {
		// TODO calculate roll, pitch, and yaw from imu data
		
		double rollA = Math.atan2(aX, Math.sqrt(Math.pow(aY,2) + Math.pow(aZ,2)));
		double pitchA = Math.atan2(aY, Math.sqrt(Math.pow(aX,2) + Math.pow(aZ,2)));
		
		double rollR = Math.atan2(rY, Math.sqrt(Math.pow(rX,2) + Math.pow(rZ,2)));
		double pitchR = Math.atan2(-rX, rZ);
		
		double yaw = 0.0;
		
		//msgPub.publish(new ImuMeasurement());
	}*/
	

	@Override
	public boolean matchDataSample(byte[] sample) {
		return true;
	}

	@Override
	public int matchDataMinSize() {
		return 0;
	}

	@Override
	public int baudRate() {
		return 57600;
	}

	@Override
	public int peel(byte[] buffer, int start, int bytes_available) {
		// TODO replace 80 with max message length
		if(bytes_available < 30) {
			// Not enough bytes...maybe?
			return 0;
		}
		
		// Check the prefix. was previously #ACG
		if(buffer[start] != '#') {
			return 1;
		}
		if(buffer[start+1] != 'Y') {
			return 1;
		}
		if(buffer[start+2] != 'P') {
			return 1;
		}
		if(buffer[start+3] != 'R') {
			return 1;
		}
		if(buffer[start+4] != '=') {
			return 1;
		}
		
		
		
		
		double[] vals = new double[3];
		String b = new String(buffer, start+5, bytes_available-5);
		int orig_length = b.length();
		for (int i = 0; i < 2; i++) {
			// TODO: need less than bytes_availble
			int comma_index = b.indexOf(',');
			try {
				Double d = Double.parseDouble(b.substring(0, comma_index));
				vals[i] = d;
			} catch (NumberFormatException nfe) {
				System.out.println("maligned input; skipping...");
				return 1;
			}
			b = b.substring(comma_index+1);	
		}
		
		// The last one, we use the hash as the symbol!
		int hash_index = b.indexOf('#');
		vals[2] = Double.parseDouble(b.substring(0, hash_index));
		b = b.substring(hash_index);	
			
		msgPub.publish(new ImuMeasurement(0, 0, 0, 
				0, 0, 0, vals[0], vals[1], vals[2]));
		return 4 + (orig_length - b.length());
	}
}