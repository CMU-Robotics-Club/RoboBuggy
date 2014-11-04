package com.roboclub.robobuggy.sensors;

import gnu.io.SerialPortEvent;

import com.roboclub.robobuggy.main.Robot;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.ros.Publisher;
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

public class Gps extends SerialConnection implements Sensor{
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
	//how long the system should wait until a sensor switches to Disconnected
	private static final long SENSOR_TIME_OUT = 5000;
	
	public double lat;
	public double lon;

	public Gps(String publishPath) {
		super("GPS", BAUDRATE, HEADER);
		publisher = new Publisher(publishPath);
		thisSensorType = SensorType.GPS;
	}
	
	public long timeOfLastUpdate(){
		return lastUpdateTime;
	}
	
	private float parseLat(String latNum) {
		return (Float.valueOf(latNum.substring(0,2)) + 
				(Float.valueOf(latNum.substring(2)) / 60.0f));
	}
	
	private float parseLon(String lonNum) {
		return (Float.valueOf(lonNum.substring(0,3)) + 
				(Float.valueOf(lonNum.substring(3)) / 60.0f));
	}
	

	public boolean reset(){
		//TODO
		return false;
	}
	
	@Override
	public SensorState getState() {
		if(System.currentTimeMillis() - lastUpdateTime > SENSOR_TIME_OUT){
			currentState = SensorState.DISCONECTED;
		} 
		
		return currentState;
	}

	@Override
	public SensorType getSensorType() {
		return thisSensorType;
	}
	
	@Override
	public void publish() {
		float latitude = 0, longitude = 0;
		int state = 0;
		String val = "";
		
		currentState = SensorState.ON;
		lastUpdateTime = System.currentTimeMillis();
		try {
			for (int i = 0; i < index; i++) {
				if (inputBuffer[i] == '\n' || inputBuffer[i] == ',' || i == index) {
					switch (state) {
					case LAT_NUM:
						latitude =  parseLat(val);
						break;
					case LAT_DIR:
						if (val.equalsIgnoreCase("S")) latitude = -1 * latitude;
						break;
					case LONG_NUM:
						longitude = parseLon(val);
						break;
					case LONG_DIR:
						if (val.equalsIgnoreCase("W")) longitude = -1 * longitude;
						publisher.publish(new GpsMeasurement(latitude, longitude));
						Robot.UpdateGps(latitude, longitude);
						System.out.println("lat: " + latitude + " lon: " + longitude);
						lat = latitude;
						lon = longitude;
						return;
					}
					
					val  = "";
					state++;
				} else val += inputBuffer[i];
			}
		} catch (Exception e) {
			System.out.println("Failed to parse GPS message!");
			currentState = SensorState.ERROR;
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
					if (data == HEADER.charAt(index)) index++;
					else index = 0;
					
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
