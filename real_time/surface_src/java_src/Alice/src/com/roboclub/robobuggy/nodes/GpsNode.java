package com.roboclub.robobuggy.nodes;

import gnu.io.SerialPortEvent;

import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.messages.StateMessage;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.sensors.Sensor;
import com.roboclub.robobuggy.sensors.SensorState;
import com.roboclub.robobuggy.sensors.SensorType;
import com.roboclub.robobuggy.serial.SerialConnection;

/**
 * 
 * @author Kevin Brennan
 *
 * @version 0.5
 * 
 *          CHANGELOG: NONE
 * 
 *          DESCRIPTION: TODO
 */

public class GpsNode extends SerialConnection implements Sensor {
	/* Constants for Serial Communication */
	/** Header for picking correct serial port */
	private static final String HEADER = "$GPGGA";
	/** Baud rate for serial port */
	private static final int BAUDRATE = 9600;
	private static final int MESSAGE_SIZE = 40;
	/** Index of latitude data as received during serial communication */
	private static final int LAT_NUM = 2;
	/** Index of latitude direction as received during serial communication */
	private static final int LAT_DIR = 3;
	/** Index of longitude data as received during serial communication */
	private static final int LONG_NUM = 4;
	/** Index of longitude direction as received during serial communication */
	private static final int LONG_DIR = 5;
	// how long the system should wait until a sensor switches to Disconnected
	private static final long SENSOR_TIME_OUT = 5000;

	public GpsNode(SensorChannel sensor) {
		super("GPS", BAUDRATE, HEADER, sensor.getRstPath());
		msgPub = new Publisher(sensor.getMsgPath());
		statePub = new Publisher(sensor.getStatePath());
		sensorType = SensorType.GPS;

		statePub.publish(new StateMessage(this.currState));
	}

	public long timeOfLastUpdate() {
		return lastUpdateTime;
	}

	private float parseLat(String latNum) {
		return (Float.valueOf(latNum.substring(0, 2)) + (Float.valueOf(latNum
				.substring(2)) / 60.0f));
	}

	private float parseLon(String lonNum) {
		return (Float.valueOf(lonNum.substring(0, 3)) + (Float.valueOf(lonNum
				.substring(3)) / 60.0f));
	}

	@Override
	public SensorState getState() {
		if (System.currentTimeMillis() - lastUpdateTime > SENSOR_TIME_OUT) {
			System.out.println("Here");
			currState = SensorState.DISCONNECTED;
			this.currState = SensorState.ERROR;
			statePub.publish(new StateMessage(this.currState));
		}

		return currState;
	}

	@Override
	public SensorType getSensorType() {
		return sensorType;
	}

	public void publish() {
		float latitude = 0, longitude = 0;
		int state = 0;
		String val = "";

		lastUpdateTime = System.currentTimeMillis();
		try {
			for (int i = 0; i < index; i++) {
				if (inputBuffer[i] == '\n' || inputBuffer[i] == ','
						|| i == index) {
					switch (state) {
					case LAT_NUM:
						latitude = parseLat(val);
						break;
					case LAT_DIR:
						if (val.equalsIgnoreCase("S"))
							latitude = -1 * latitude;
						break;
					case LONG_NUM:
						longitude = parseLon(val);
						break;
					case LONG_DIR:
						if (val.equalsIgnoreCase("W"))
							longitude = -1 * longitude;
						msgPub.publish(new GpsMeasurement(
						return;
					}

					val = "";
					state++;
				} else
					val += inputBuffer[i];
			}
		} catch (Exception e) {
			// System.out.println("Failed to parse GPS message!");
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

	/* %***** Serial Methods *****% */
	@Override
	public void serialEvent(SerialPortEvent event) {
		switch (event.getEventType()) {
		case SerialPortEvent.DATA_AVAILABLE:
			try {
				char data = (char) input.read();

				switch (state) {
				case 0:
					if (data == HEADER.charAt(index))
						index++;
					else
						index = 0;

					if (index == HEADER.length()) {
						index = 0;
						state++;
					}
					break;
				case 1:
					inputBuffer[index++] = data;

					if (index > MESSAGE_SIZE) {
						publish();
						index = 0;
						state = 0;
					}
					break;
				}
			} catch (Exception e) {
				System.out.println("Imu Exception in port: " + this.getName());
			}
		}
	}

	@Override
	protected void serialWrite(byte[] data) {
		// TODO Auto-generated method stub
	}

	@Override
	protected void serialWrite(String data) {
		// TODO Auto-generated method stub
	}

	@Override
	public boolean isConnected() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean close() {
		// TODO Auto-generated method stub
		return false;
	}
}
