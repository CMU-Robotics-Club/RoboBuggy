package com.roboclub.robobuggy.messages;

import com.roboclub.robobuggy.ros.Message;

import java.util.Date;

/**
 * {@link Message} for passing information about the current state of the robot
 * 
 * @author Zachary Dawson
 *
 */
public class GPSPoseMessage extends BaseMessage {

	public static final String VERSION_ID = "pose_message";
	
	private final double latitude;
	private final double longitude;
	private final double heading;

	/**
	 * Constructs a new {@link GPSPoseMessage}
	 * @param timestamp {@link Date} representing the creation time
	 * @param latitude of the buggy (negative is South)
	 * @param longitude of the buggy (negative is West)
	 * @param heading of the buggy (in degrees from North)
	 */
	public GPSPoseMessage(Date timestamp, double latitude, double longitude, double heading) {
		this.latitude = latitude;
		this.longitude = longitude;
		this.heading = heading;
		this.timestamp = new Date(timestamp.getTime()).getTime();
	}
	
	/**
	 * Returns the latitude of the {@link GPSPoseMessage} (negative is South)
	 * @return the latitude of the {@link GPSPoseMessage} (negative is South)
	 */
	public double getLatitude() {
		return latitude;
	}
	
	/**
	 * Returns the longitude of the {@link GPSPoseMessage} (negative is West)
	 * @return the longitude of the {@link GPSPoseMessage} (negative is West)
	 */
	public double getLongitude() {
		return longitude;
	}
	
	/**
	 * Returns the heading of the {@link GPSPoseMessage} (in degrees from North)
	 * @return the heading of the {@link GPSPoseMessage} (in degrees from North)
	 */
	public double getHeading() {
		return heading;
	}
	
	/**
	 * evaluates to the distance between two gps points based on an L2 metric
	 * @param a the first gps point
	 * @param b the second gps point
	 * @return the distince 
	 */
	public static double getDistance(GPSPoseMessage a, GPSPoseMessage b){
		double dx = a.getLongitude() - b.getLongitude();
		double dy = a.getLatitude() - b.getLatitude();
		return Math.sqrt(dx*dx + dy*dy);
	}

}
