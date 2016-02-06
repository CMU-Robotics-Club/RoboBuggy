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
	public LocTuple(double x, double y){
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
	
	/**
	 * Determines the magnitude of the difference between two {@link LocTuple} 
	 * objects (in degrees)
	 * @param loc {@link LocTuple} to compare to
	 * @return difference between the two {@link LocTuple} objects (in degrees)
	 */
	public double getMagDiff(LocTuple loc) {
		return Math.sqrt(Math.pow(this.latitude-loc.latitude, 2) +
				Math.pow(this.longitude-loc.longitude, 2));
	}
	
	/**
	 * Returns the clockwise angle (in degrees) of the location from the 
	 * positive y-axis (North)
	 * @return clockwise angle (in degrees)
	 */
	public double getHeadingAngle() {
		return ((Math.toDegrees(Math.atan2(latitude, longitude))-90.0)+360.0)%360.0;
	}
	
	/**
	 * Subtracts two {@link LocTuple}s
	 * @param lt1 {@link LocTuple} 1
	 * @param lt2 {@link LocTuple} 2
	 * @return {@link LocTuple} representing lt1-lt2
	 */
	public static LocTuple subtract(LocTuple lt1, LocTuple lt2) {
		double lat = lt1.latitude - lt2.latitude;
		double lon = lt1.longitude - lt2.longitude;
		return new LocTuple(lat, lon);
	}
}