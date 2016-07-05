package com.roboclub.robobuggy.nodes.localizers;

/**
 * Motion model - TODO
 */
public class MotionModel {
	/** @brief distance between buggy axles */
	private static final double WHEELBASE_M = 1.13;

	MotionModel(){
		//TODO
		

	}
	
	/**
	 * Project our path around a circle of our turning radius to find change in heading.
	 * @param deltaDistance Distance traveled since last heading update. Assumed to be along constant radius arc.
	 * @param steeringAngle the current steering angle 
	 * @return the heading change
	 */
	public static double getHeadingChange(double deltaDistance, double steeringAngle) {
		double turningRadius = WHEELBASE_M / Math.sin(Math.toRadians(steeringAngle));
		return (deltaDistance * 360) / (2*Math.PI*turningRadius);
	}
	
}
