package com.roboclub.robobuggy.ui;


/**
 * Private class used for representing a latitude and longitude combination
 */
public class LocTuple {
	private double latitude;
	private double longitude;
	
	/**
	 * Construct a new {@link LocTuple}
	 * @param x latitude
	 * @param y longitude
	 */
	LocTuple(double x, double y){
		this.latitude = x;
		this.longitude = y;
	}
	
	/**
	 * Returns the latitude of the {@link LocTuple}
	 * @return the latitude of the {@link LocTuple}
	 */
	public double getLatitude(){
		return latitude;
	}
	
	/**
	 * Returns the longitude of the {@link LocTuple}
	 * @return the longitude of the {@link LocTuple}
	 */
	public double getLongitude(){
		return longitude;
	}
}