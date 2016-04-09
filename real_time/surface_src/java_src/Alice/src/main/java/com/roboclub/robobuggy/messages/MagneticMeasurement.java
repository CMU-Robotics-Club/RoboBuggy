package com.roboclub.robobuggy.messages;

import java.util.Date;

/**
 * Message for passing IMU magnetic north message within BuggyROS
 * @author ?
 *
 * @version 0.5
 * 
 *          CHANGELOG: NONE
 * 
 *          DESCRIPTION: TODO
 */
public class MagneticMeasurement extends BaseMessage {

	public static final String VERSION_ID = "magV0.0";


	private double magX;
	private double magY;
	private double magZ;
	
	/**
	 * Constructs a new {@link MagneticMeasurement} at time now
	 * @param magX magX value
	 * @param magY magY value
	 * @param magZ magZ value
	 */
	public MagneticMeasurement(double magX, double magY, double magZ) {
		this.timestamp = new Date().getTime();
		this.magX = magX;
		this.magY = magY;
		this.magZ = magZ;
	}
	
	/**
	 * Returns the x value of the {@link MagneticMeasurement}
	 * @return the x value of the {@link MagneticMeasurement}
	 */
	public double getmagX() {
		return magX;
	}
	
	/**
	 * Returns the y value of the {@link MagneticMeasurement}
	 * @return the y value of the {@link MagneticMeasurement}
	 */
	public double getmagY() {
		return magY;
	}
	
	/**
	 * Returns the z value of the {@link MagneticMeasurement}
	 * @return the z value of the {@link MagneticMeasurement}
	 */
	public double getmagZ() {
		return magZ;
	}

}
