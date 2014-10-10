package com.roboclub.robobuggy.sensors;

import com.roboclub.robobuggy.main.Robot;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.serial.SerialConnection;
import com.roboclub.robobuggy.serial.SerialEvent;
import com.roboclub.robobuggy.serial.SerialListener;
import com.roboclub.robobuggy.ui.Gui;

public class GPS extends SerialConnection implements Sensor{
	/* Constants for Serial Communication */
	/** Header for picking correct serial port */
	private static final String HEADER = "$GPGGA";
	/** Baud rate for serial port */
	private static final int BAUDRATE = 9600;
	/** Index of latitude data as received during serial communication */
	private static final int LAT_NUM = 1;
	/** Index of latitude direction as received during serial communication */
	private static final int LAT_DIR = 2;
	/** Index of longitude data as received during serial communication */
	private static final int LONG_NUM = 3;
	/** Index of longitude direction as received during serial communication */
	private static final int LONG_DIR = 4;
	
	private float latitude;
	private float longitude;
	
	private Publisher gpsPub;

	public GPS(String publishPath) {
		super("GPS", BAUDRATE, HEADER);
		super.addListener(new GpsListener());
		System.out.println("Initializing GPS");
		gpsPub = new Publisher(publishPath);
	}
	
	private float parseLat(String latNum) {
		return (Float.valueOf(latNum.substring(0,2)) + 
				(Float.valueOf(latNum.substring(2)) / 60.0f));
	}
	
	private float parseLon(String lonNum) {
		return (Float.valueOf(lonNum.substring(0,3)) + 
				(Float.valueOf(lonNum.substring(3)) / 60.0f));
	}
	
	/**
	 * GpsListener is an event handler for incoming serial messages. It
	 * is notified every time a complete serial message is received by
	 * the serial port for the given GpsPanel.
	 */
	private class GpsListener implements SerialListener {
		@Override
		public void onEvent(SerialEvent event) {
			char[] tmp = event.getBuffer();
			int length = event.getLength();
			int index = 0;	
			
			if (tmp != null && event.getLength() > HEADER.length()) {
				String curVal = "";
				
				// Filter messages by header
				for (int i = 0; i < HEADER.length(); i++) {
					if (tmp[i] != HEADER.charAt(i)) return;
				}
				
				// Parse message data
				for (int i = HEADER.length()+1; i < length; i++ ) {
					if (tmp[i] == ',' || tmp[i] == '\n') {
						try {
							switch ( index ) {
							case LAT_NUM:
								latitude =  parseLat(curVal);
								break;
							case LAT_DIR:
								if (curVal.equalsIgnoreCase("S")) latitude = -1 * latitude;
								break;
							case LONG_NUM:
								longitude = parseLon(curVal);
								break;
							case LONG_DIR:
								if (curVal.equalsIgnoreCase("W")) longitude = -1 * longitude;
								gpsPub.publish(new GpsMeasurement(latitude, longitude));
								return;
							}
							
							curVal = "";
							index++;
						} catch (Exception e) {
							System.out.println("Failed to parse gps message");
							return;
						}
						
					} else {
						curVal += tmp[i];
					}
				}
			}
		}
	}
}
