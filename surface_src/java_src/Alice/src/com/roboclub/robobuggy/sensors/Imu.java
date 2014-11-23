package com.roboclub.robobuggy.sensors;

import gnu.io.SerialPortEvent;

import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.messages.StateMessage;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.serial.SerialConnection;

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

public class Imu extends SerialConnection implements Sensor {
	/** Header for choosing serial port */
	private static final String HEADER = "#ACG=";
	/** Baud rate for serial port */
	private static final int BAUDRATE = 57600;
	/** Index of accel x data as received during serial communication */
	private static final int AX = 0;
	/** Index of accel y data as received during serial communication */
	private static final int AY = 1;
	/** Index of accel z data as received during serial communication */
	private static final int AZ = 2;
	/** Index of gyro x data as received during serial communication */
	private static final int RX = 3;
	/** Index of gyro y data as received during serial communication */
	private static final int RY = 4;
	/** Index of gyro z data as received during serial communication */
	private static final int RZ = 5;
	/** Index of magnetometer x data as received during serial communication */
	private static final int MX = 6;
	/** Index of magnetometer y data as received during serial communication */
	private static final int MY = 7;
	/** Index of magnetometer z data as received during serial communication */
	private static final int MZ = 8;
	// how long the system should wait until a sensor switches to Disconnected
	private static final long SENSOR_TIME_OUT = 5000;

	private static final double GYRO_PER = 0.9;
	private static final double ACCEL_PER = 1 - GYRO_PER;
	
	public double angle;
	
	public Imu(SensorChannel sensor) {
		super("IMU", BAUDRATE, HEADER, sensor.getRstPath());
		msgPub = new Publisher(sensor.getMsgPath());
		statePub = new Publisher(sensor.getStatePath());
		sensorType = SensorType.IMU;
		
		statePub.publish(new StateMessage(this.currState));
	}

	public boolean reset(){
		//TODO
		return false;
	}
	
	@Override
	public long timeOfLastUpdate() {
		return lastUpdateTime;
	}

	@Override
	public SensorState getState() {
		if (System.currentTimeMillis() - lastUpdateTime > SENSOR_TIME_OUT) {
			currState = SensorState.ERROR;
			statePub.publish(new StateMessage(this.currState));
		} 
		return currState;
	}

	@Override
	public SensorType getSensorType() {
		return sensorType;
	}
	
	@Override
	public boolean isConnected() {
		return this.connected;
	}

	@Override
	public boolean close() {
		if (connected) {
			try {
				input.close();
				output.close();
				port.close();
				return true;
			} catch (Exception e) {
				System.out.println("Failed to Close Port: " + this.getName());
			}
		}
		
		return false;
	}

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
	}
	
	@Override
	public void publish() {
		double aX = 0, aY = 0, aZ = 0, 
				rX = 0, rY = 0, rZ = 0,
				mX = 0, mY = 0, mZ = 0;
		String val = "";
		int state = 0;

		lastUpdateTime = System.currentTimeMillis();

		try {
			for (int i = 0; i < index; i++) {
				if (inputBuffer[i] == '\n' || inputBuffer[i] == ',' || i == index) {
					switch (state) {
					case AX:
						aX = Double.valueOf(val);
						break;
					case AY:
						aY = Double.valueOf(val);
						break;
					case AZ:
						aZ = Double.valueOf(val);
						break;
					case RX:
						rX = Double.valueOf(val);
						break;
					case RY:
						rY = Double.valueOf(val);
						break;
					case RZ:
						rZ = Double.valueOf(val);
						break;
					case MX:
						mX = Double.valueOf(val);
						break;
					case MY:
						mY = Double.valueOf(val);
						break;
					case MZ:
						mZ = Double.valueOf(val);
						//TODO estimateOrientation(aX, aY, aZ, rX, rY, rZ, mX, mY, mZ);
						
						msgPub.publish(new ImuMeasurement(aX, aY, aZ, rX, rY,
								rZ, mX, mY, mZ));
						break;
					}
					
					val = "";
					state++;
				} else {
					val += inputBuffer[i];
				}
			}
		} catch (Exception e) {
			System.out.println("Failed to parse Imu Message");
			if (this.currState != SensorState.FAULT) {
				this.currState = SensorState.FAULT;
				statePub.publish(new StateMessage(this.currState));
			}
			return;
		}
		
		if (this.currState != SensorState.ON) {
			this.currState = SensorState.ON;
			statePub.publish(new StateMessage(this.currState));
		}
	}
	
	/*%*****		Serial Methods			*****%*/
	@Override
	protected void serialWrite(byte[] data) {
		if (connected && output != null) {
			try {
				output.write(data);
				System.out.println("Wrote: " + data);
				//output.flush();
			} catch (Exception e) {
				System.out.println("Unable to write: " + data);
			}
		}
	} 

	@Override
	protected void serialWrite(String data) {
		if (connected && data != null && output != null) {
			try {
				output.write(data.getBytes());
				output.flush();
			} catch (Exception e) {
				System.out.println("Unable to write: " + data);
			}
		}
	}
	
	byte[] bigBuffer = new byte[1024];
	public void serialEvent(SerialPortEvent event) {
		int num_read = -1;
		switch (event.getEventType()) {
		case SerialPortEvent.DATA_AVAILABLE:
			try {
				num_read = input.read(bigBuffer);
			} catch (Exception e) {
				System.out.println(this.getName() + " exception!");
				e.printStackTrace();
			}
			for(int i = 0; i < num_read; i++) {
				char data = (char) bigBuffer[i];
				switch (state) {
				case 0:
					if (data == HEADER.charAt(index)) {
						index++;
					} else {
						index = 0;
					}

					if (index == HEADER.length()) {
						index = 0;
						state++;
					}
					break;
				case 1:
					inputBuffer[index++] = data;

					if (data == '\n' || index >= BUFFER_SIZE) {
						publish();
						index = 0;
						state = 0;
					}
				}

			}
			break;
		default:
			break;
		}
	}
	}
