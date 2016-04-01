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


	private double rotationX;
	private double rotationY;
	private double rotationZ;

	/**
	 * Constructs a new {@link MagMeasurement} at time now
	 * @param rotationX rotationx value
	 * @param rotationy rotationy value
	 * @param rotationz rotationz value
	 */
	public MagneticMeasurement(double rotationX, double rotationY, double rotationZ) {
		this.timestamp = new Date().getTime();
		this.rotationX = rotationX;
		this.rotationY = rotationY;
		this.rotationZ = rotationZ;
	}
	
	/**
	 * Returns the rotationX value of the {@link MagMeasurement}
	 * @return the rotationX value of the {@link MagMeasurement}
	 */
	public double getRotationX() {
		return rotationX;
	}
	
	/**
	 * Returns the rotationY value of the {@link MagMeasurement}
	 * @return the rotationY value of the {@link MagMeasurement}
	 */
	public double getRotationY() {
		return rotationY;
	}
	
	/**
	 * Returns the rotationZ value of the {@link MagMeasurement}
	 * @return the rotationZ value of the {@link MagMeasurement}
	 */
	public double getRotationZ() {
		return rotationZ;
	}

}
