package com.roboclub.robobuggy.sensors;

import com.roboclub.robobuggy.main.Robot;
import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.serial.SerialConnection;
import com.roboclub.robobuggy.serial.SerialEvent;
import com.roboclub.robobuggy.serial.SerialListener;

public class Imu extends SerialConnection implements Sensor {
	/* Constants for serial communication */
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

	private SensorType thisSensorType;

	
	long lastUpdateTime;

	private float aX;
	private float aY;
	private float aZ;
	private float rX;
	private float rY;
	private float rZ;
	private float mX;
	private float mY;
	private float mZ;

	private Publisher imuPub;

	private SensorState currentState;

	public Imu(String publishPath) {
		super("IMU", BAUDRATE, HEADER);
		super.addListener(new ImuListener());

		imuPub = new Publisher("/sensor/IMU");

		System.out.println("Initializing IMU");

	}

	public boolean reset(){
		//TODO
		return false;
	}
	
	@Override
	public long timeOfLastUpdate() {
		return lastUpdateTime;
	}

	private void setValue(int index, float value) {
		switch (index) {
		case AX:
			aX = value;
			break;
		case AY:
			aY = value;
			break;
		case AZ:
			aZ = value;
			break;
		case RX:
			rX = value;
			break;
		case RY:
			rY = value;
			break;
		case RZ:
			rZ = value;
			break;
		case MX:
			mX = value;
			break;
		case MY:
			mY = value;
			break;
		case MZ:
			mZ = value;
			Robot.UpdateImu(aX, aY, aZ, rX, rY, rZ, mX, mY, mZ);
			imuPub.publish(new ImuMeasurement(aX, aY, aZ, rX, rY, rZ, mX, mY,
					mZ));
			System.out.format(
					"IMU Values: aX: %f aY: %f aZ: %f rX: %f rY: %f rZ: %f "
							+ "mX: %f mY: %f mZ: %f \n", aX, aY, aZ, rX, rY,
					rZ, mX, mY, mZ);
			break;
		default:
			return;
		}
	}

	/**
	 * ImuListener is an event handler for serial communication. It is notified
	 * every time a complete message is received by serial port for the given
	 * panel. It handles the serial event and parses the data to update the
	 * current properties of the given panel.
	 */
	private class ImuListener implements SerialListener {
		@Override
		// TODO add avilable and on state update
		public void onEvent(SerialEvent event) {
			char[] tmp = event.getBuffer();
			int index = 0;
			boolean valid = true;
			if (tmp != null && event.getLength() > HEADER.length()) {
				String curVal = "";
				for (int i = HEADER.length(); i < event.getLength(); i++) {
					if (tmp[i] == '\n') {
						if (valid) {
							if (currentState == SensorState.ON) {
								currentState = SensorState.ON;
							} else {
								currentState = SensorState.AVILABLE;
							}
							lastUpdateTime = System.currentTimeMillis();
						}
						break;
					} else if (tmp[i] == ',') {
						try {
							setValue(index, Float.valueOf(curVal));

							curVal = "";
							index++;
						} catch (Exception e) {
							System.out.println("Failed to parse IMU message");
							currentState = SensorState.ERROR;
							lastUpdateTime = System.currentTimeMillis();
							valid = false;
							return;
						}

					} else {
						curVal += tmp[i];
					}
				}
			}
		}
	}

	@Override
	public SensorState getState() {
		if (System.currentTimeMillis() - lastUpdateTime > SENSOR_TIME_OUT) {
			currentState = SensorState.DISCONECTED;
		}
		return currentState;
	}

	@Override
	public SensorType getSensorType() {
		return thisSensorType;
	}
}
