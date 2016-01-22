package com.roboclub.robobuggy.nodes.sensors;

import java.util.Arrays;
import java.util.Date;

import com.orsoncharts.util.json.JSONObject;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.messages.StateMessage;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyBaseNode;
import com.roboclub.robobuggy.nodes.baseNodes.NodeState;
import com.roboclub.robobuggy.nodes.baseNodes.SerialNode;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.NodeChannel;

/**
 * {@link SerialNode} for reading in GPS data
 * @author Matt Sebek 
 * @author Kevin Brennan
 *
 */
public final class GpsNode extends SerialNode {
	// how long the system should wait until a sensor switches to Disconnected
	private static final long SENSOR_TIME_OUT = 5000;
	
	private static final int BAUD_RATE = 9600;

	// Used for state estimation
	/*private static final double GYRO_PER = 0.9;
	private static final double ACCEL_PER = 1 - GYRO_PER;
	public double angle;
	*/

	private Publisher msgPub;
	private Publisher statePub;
	
	/**
	 * Creates a new {@link GpsNode}
	 * @param sensor {@link NodeChannel} of GPS
	 * @param portName name of the serial port to read from
	 */
	public GpsNode(NodeChannel sensor, String portName) {
		super(new BuggyBaseNode(sensor), "GPS", portName, BAUD_RATE);
		msgPub = new Publisher(sensor.getMsgPath());
		statePub = new Publisher(sensor.getStatePath());
	
		statePub.publish(new StateMessage(NodeState.DISCONNECTED));
	}

	/**{@inheritDoc}*/
	@Override
	public boolean matchDataSample(byte[] sample) {
		return true;
	}

	/**{@inheritDoc}*/
	@Override
	public int matchDataMinSize() {
		return 0;
	}

	/**{@inheritDoc}*/
	@Override
	public int getBaudRate() {
		return BAUD_RATE;
	}
	
	private Date convertHHMMSStoTime(String timeHHMMSSss) {
		Date d = new Date();
		float timeHH = Float.parseFloat(timeHHMMSSss.substring(0, 2));
		float timeMM = Float.parseFloat(timeHHMMSSss.substring(2, 4));
		float timeSS = Float.parseFloat(timeHHMMSSss.substring(4));
		d.setHours((int) timeHH);
		d.setMinutes((int) timeMM);
		d.setSeconds((int) timeSS);
		return d;
	}

	private double convertMinutesSecondsToFloat(String posDDMMmmmmm) {
		double posDD = Double.parseDouble(posDDMMmmmmm.substring(0, 2));
		double posMM = Double.parseDouble(posDDMMmmmmm.substring(2));
		return posDD + posMM/60.0;
	}
	
	private double convertMinSecToFloatLongitude(String dddmmmmm) {
		double ddd = Double.parseDouble(dddmmmmm.substring(0, 3));
		double mins = Double.parseDouble(dddmmmmm.substring(3));
		return ddd + mins / 60.0;
	}
	
	/**{@inheritDoc}*/
	@Override
	public int peel(byte[] buffer, int start, int bytesAvailable) {
		// TODO replace 80 with max message length
		// This lets us avoid handling arcane failure cases about not-enough message.
		if(bytesAvailable < 80) {
			// Not enough bytes...maybe?
			return 0;
		}

		String str = Arrays.toString(buffer).substring(start, bytesAvailable);

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
			setNodeState(NodeState.ERROR);
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
		double longitude = convertMinSecToFloatLongitude(ar[4]);
		boolean west;
		if(ar[5].equals("W")) {
			west = true;
		} else if(ar[5].equals("E")) {
			west = false;
		} else {
			System.out.println("uhoh, you can't go not north or south!");
			throw new RuntimeException();
		}
		
		
		int numSatellites = Integer.parseInt(ar[7]);
		double horizontalDilutionOfPrecision = Double.parseDouble(ar[8]);
		double antennaAltitude = Double.parseDouble(ar[9]);
		
		msgPub.publish(new GpsMeasurement(readingTime, latitude, north, longitude, 
			west, quality, numSatellites, horizontalDilutionOfPrecision, antennaAltitude,
			Double.parseDouble(ar[2]), Double.parseDouble(ar[4])));
		
		//Feed the watchdog
		setNodeState(NodeState.ON);
		return ar[0].length() + ar[1].length() + ar[2].length() + ar[3].length() 
				+ ar[4].length() + ar[5].length() + ar[6].length() + ar[7].length()
				+ ar[8].length() + ar[9].length();
	}

	/**
	 * Called to translate a peeled message to a JSON object
	 * @param message {@link String} of the peeled message
	 * @return {@link JSONObject} representing the string
	 */
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
		params.put("raw_gps_lat", Double.valueOf(messageData[11]));
		params.put("raw_gps_lon", Double.valueOf(messageData[12]));
		data.put("timestamp", messageData[1]);
		data.put("name", "GPS");
		data.put("params", params);
		return data;
	}
}