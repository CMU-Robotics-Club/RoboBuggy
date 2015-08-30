package org.roboclub.robobuggy.localization;

import java.sql.Time;

/**
 * 
 * @author Trevor Decker
 * @version 0.0
 *
 * Description of interface for a "localization filter" or "localization system" api
 * This will allow for the method of sensor fusion and pose estimation to be changed
 * without required changes to the rest of the code base. 
 * For example a particle filter could be used instead of a kalman filter. 
 * TODO implement
 */
public interface LocalizationFilter {

	/**
	 * TODO document
	 * @param aMeasurment
	 */
	void addSensorMeaserment(SensorMeasurment aMeasurment); 
	
	/**
	 * TODO document 
	 * @param atime
	 * @return
	 */
	LocationEstimate getEstimateAt(Time atime);
	
	/**
	 * TODO document
	 * @param initState
	 */
	void reInitSystem(LocationEstimate initState);
	
}
