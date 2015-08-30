package org.roboclub.robobuggy.localization;

import java.sql.Time;

/**
 * TODO implement
 * @author Trevor Decker
 * @version 0.0
 * 
 *  A locilization filter that trusts the most recent measurement completely 
 *
 */
public class HighTrustFilter implements LocalizationFilter{

	@Override
	/**
	 * TODO implement
	 * TODO document 
	 */
	public void addSensorMeaserment(SensorMeasurment aMeasurment) {
		// TODO Auto-generated method stub
		
	}

	@Override
	/**
	 * TODO implement
	 * TODO document
	 */
	public LocationEstimate getEstimateAt(Time atime) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	/**
	 * TODO implement
	 * TODO document 
	 */
	public void reInitSystem(LocationEstimate initState) {
		// TODO Auto-generated method stub
		
	}

}
