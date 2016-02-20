package com.roboclub.robobuggy.messages;

import com.roboclub.robobuggy.ros.Message;

import java.util.Date;

/**
 * Message for passing IMU measurements within BuggyROS
 * @author ?
 *
 * @version 0.5
 * 
 *          CHANGELOG: NONE
 * 
 *          DESCRIPTION: TODO
 */
public class ImuMeasurement extends BaseMessage {

	public static final String VERSION_ID = "imuV0.0";


	private double yaw;
	private double pitch;
	private double roll;

	/**
	 * Constructs a new {@link ImuMeasurement} at time now
	 * @param y yaw value
	 * @param p pitch value
	 * @param r roll value
	 */
	public ImuMeasurement(double y, double p, double r) {
		this.timestamp = new Date().getTime();
		this.yaw = y;
		this.pitch = p;
		this.roll = r;
	}
	
	/**
	 * Returns the yaw value of the {@link ImuMeasurement}
	 * @return the yaw value of the {@link ImuMeasurement}
	 */
	public double getYaw() {
		return yaw;
	}
	
	/**
	 * Returns the pitch value of the {@link ImuMeasurement}
	 * @return the pitch value of the {@link ImuMeasurement}
	 */
	public double getPitch() {
		return pitch;
	}
	
	/**
	 * Returns the roll value of the {@link ImuMeasurement}
	 * @return the roll value of the {@link ImuMeasurement}
	 */
	public double getRoll() {
		return roll;
	}

	/**{@inheritDoc}*/
	@Override
	public String toLogString() {
		String s = formatDate(timestamp);
		return s + "," + Double.toString(yaw) + "," 
				+ Double.toString(pitch) + "," 
				+ Double.toString(roll);
	}

	/**{@inheritDoc}*/
	@Override
	public Message fromLogString(String str) {
		String delims = ",";
		String[] ar = str.split(delims);

		// Creating SimpleDateFormat with yyyyMMdd format e.g."20110914"
		String yyyyMMdd = ar[0];
		timestamp = tryToParseDate(yyyyMMdd).getTime();

		Double y = Double.parseDouble(ar[1]);
		Double p = Double.parseDouble(ar[2]);
		Double r = Double.parseDouble(ar[3]);
		return new ImuMeasurement(y, p, r);
	}
}
