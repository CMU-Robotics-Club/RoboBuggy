package com.roboclub.robobuggy.nodes;

import java.text.SimpleDateFormat;
import java.util.Date;

import jdk.nashorn.internal.scripts.JS;

import com.orsoncharts.util.json.JSONObject;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.messages.StateMessage;
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.ros.Subscriber;
import com.roboclub.robobuggy.sensors.SensorState;
import com.roboclub.robobuggy.serial.SerialNode;

/**
 * @author Matt Sebek 
 * @author Kevin Brennan
 *
 */

public class GpsNode extends SerialNode implements Node {
	// how long the system should wait until a sensor switches to Disconnected
	private static final long SENSOR_TIME_OUT = 5000;

	// Used for state estimation
	/*private static final double GYRO_PER = 0.9;
	private static final double ACCEL_PER = 1 - GYRO_PER;
	public double angle;
	*/

	public Publisher msgPub;
	public Publisher statePub;
	
	public GpsNode(SensorChannel sensor) {
		super("GPS");
		//new Subscriber(SensorChannel.GPS.getMsgPath(), messageListener)
		
		msgPub = new Publisher(sensor.getMsgPath());
		statePub = new Publisher(sensor.getStatePath());
	
		statePub.publish(new StateMessage(SensorState.DISCONNECTED));
	}

	@Override
	public void setSerialPort(gnu.io.SerialPort sp) {
		super.setSerialPort(sp);
		statePub.publish(new StateMessage(SensorState.ON));
	};
	

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
		return 9600;
	}
	
	private Date convertHHMMSStoTime(String HHMMSSss) {
		Date d = new Date();
		float HH = Float.parseFloat(HHMMSSss.substring(0, 2));
		float MM = Float.parseFloat(HHMMSSss.substring(2, 4));
		float SS = Float.parseFloat(HHMMSSss.substring(4));
		d.setHours((int) HH);
		d.setMinutes((int) MM);
		d.setSeconds((int) SS);
		return d;
	}

	private double convertMinutesSecondsToFloat(String DDMMmmmmm) {
		double DD = Float.parseFloat(DDMMmmmmm.substring(0, 2));
		double MM = Float.parseFloat(DDMMmmmmm.substring(2, 4));
		double SS = Float.parseFloat(DDMMmmmmm.substring(4));
		return DD + MM/60.0 + SS / 3600.0;
	}
	
	private double convertMinSecToFloat_Longitude(String dddmmmmm) {
		double ddd = Float.parseFloat(dddmmmmm.substring(0, 3));
		double mins = Float.parseFloat(dddmmmmm.substring(3, 5));
		double secs = Float.parseFloat(dddmmmmm.substring(5));
		return ddd + mins / 60.0 + secs / 3600.0;
	}
	
	@Override
	public int peel(byte[] buffer, int start, int bytes_available) {
		// TODO replace 80 with max message length
		// This lets us avoid handling arcane failure cases about not-enough message.
		if(bytes_available < 80) {
			// Not enough bytes...maybe?
			return 0;
		}

		String str = new String(buffer, start, bytes_available);

		// Quick check to reject things without doing string stuff (which is not cheap)
		if(buffer[start] != '$') {
			return 1;
		}
		if(buffer[start+1] != 'G') {
			return 1;
		}
		if(buffer[start+2] != 'P') {
			return 1;
		}
		if(buffer[start+3] != 'G') {
			return 1;
		}
		if(buffer[start+4] != 'G') {
			return 1;
		}
		if(buffer[start+5] != 'A') {
			return 1;
		}
		
		// Check the prefix.
		String[] ar = str.split(",");
		if(!ar[0].equals("$GPGGA")) {
			System.out.println("We saw this, but then didn't see this. hmm.");
			throw new RuntimeException();
		}
		
		// Check for valid reading
		int quality = Integer.parseInt(ar[6]);
		if(quality == 0) {
			//System.out.println("No lock...");
			// TODO publish not-lock to someone
			return 1;
		}
		Date readingTime = convertHHMMSStoTime(ar[1]);
		double latitude = convertMinutesSecondsToFloat(ar[2]);

		boolean north;
		if(ar[3].equals("N")) {
			north = true;
		} else if(ar[3].equals("S")) {
			north = false;
		} else {
			System.out.println("uhoh, you can't go not north or south!");
			throw new RuntimeException();
		}
		double longitude = convertMinSecToFloat_Longitude(ar[4]);
		boolean west;
		if(ar[5].equals("W")) {
			west = true;
		} else if(ar[5].equals("E")) {
			west = false;
		} else {
			System.out.println("uhoh, you can't go not north or south!");
			throw new RuntimeException();
		}
		
		
		int num_satellites = Integer.parseInt(ar[7]);
		double horizontal_dilution_of_precision = Double.parseDouble(ar[8]);
		double antenna_altitude = Double.parseDouble(ar[9]);
		
		msgPub.publish(new GpsMeasurement(readingTime, latitude, north, longitude, 
			west, quality, num_satellites, horizontal_dilution_of_precision, antenna_altitude));
		return ar[0].length() + ar[1].length() + ar[2].length() + ar[3].length() 
				+ ar[4].length() + ar[5].length() + ar[6].length() + ar[7].length()
				+ ar[8].length() + ar[9].length();
	}

	
	@SuppressWarnings("unchecked")
	public static JSONObject translatePeelMessageToJObject(String message) {
		// TODO Auto-generated method stub
		JSONObject data = new JSONObject();
		JSONObject params = new JSONObject();
		String[] messageData = message.split(",");
		params.put("latitude", Double.valueOf(messageData[3]));
		params.put("lat_direction", messageData[4]);
		params.put("longitude", Double.valueOf(messageData[5]));
		params.put("long_direction", messageData[6]);
		params.put("gps_quality", messageData[7]);
		params.put("num_satellites", messageData[8]);
		params.put("HDOP", Double.valueOf(messageData[9]));
		params.put("antenna_altitude", Float.valueOf(messageData[10]));
		data.put("timestamp", messageData[1]);
		data.put("name", "GPS");
		data.put("params", params);
		return data;
	}
}