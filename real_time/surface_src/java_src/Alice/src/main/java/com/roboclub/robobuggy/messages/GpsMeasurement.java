package com.roboclub.robobuggy.messages;

import java.util.Date;

import com.roboclub.robobuggy.ros.Message;

/**
 * Message used to pass GPS measurements over BuggyROS
 * gpsTimestamp is the UTC time in hours for the given position.
 * See http://hemispheregnss.com/gpsreference/GPGGA.html for more information
 * @author Matt Sebek (msebek)
 *
 * @version 0.5
 * 
 *          CHANGELOG: NONE
 * 
 *          DESCRIPTION: TODO
 */
public class GpsMeasurement extends BaseMessage implements Message {
	public static final String VERSION_ID = "gpsV0.2";

	private Date timestamp;
	private Date gpsTimestamp;
	private double latitude;
	private boolean north;
	private double longitude;
	private boolean west;
	private int qualityValue;
	private int numSatellites;
	private double horizontalDilutionOfPrecision;
	private double antennaAltitude;
	private double rawGPSLat;
	private double rawGPSLong;
	
	/**
	 * Constructs a new {@link GpsMeasurement} at time now
	 * @param gpsTimestamp {@link Date} of the GPS timestamp
	 * @param latitude latitude measurement
	 * @param north true iff the latitude measurement is north
	 * @param longitude longitude measurement
	 * @param west true iff the longitude measurement is west
	 * @param qualityValue value of the signal quality
	 * @param numSatellites number of GPS satellites connected
	 * @param horizontalDilutionOfPrecision horizontal dilation of precision
	 * @param antennaAltitude altitude of the antenna
	 * @param rawGPSLat raw GPS latitude
	 * @param rawGPSLong raw GPS longitude
	 */
	public GpsMeasurement(Date gpsTimestamp, double latitude, boolean north, double longitude, 
			boolean west, int qualityValue, int numSatellites, 
			double horizontalDilutionOfPrecision, double antennaAltitude, double rawGPSLat, double rawGPSLong) {	
		this.timestamp = new Date();
		this.gpsTimestamp = gpsTimestamp;
		this.latitude = latitude;
		this.north = north;
		this.longitude = longitude;
		this.west = west;
		this.qualityValue = qualityValue;
		this.numSatellites = numSatellites;
		this.horizontalDilutionOfPrecision = horizontalDilutionOfPrecision;
		this.antennaAltitude = antennaAltitude;
		this.rawGPSLat = rawGPSLat;
		this.rawGPSLong = rawGPSLong;
	}

	/**
	 * Constructs a new {@link GpsMeasurement}
	 * @param messageTimestamp {@link Date} representing the time of the message
	 * @param gpsTimestamp {@link Date} of the GPS timestamp
	 * @param latitude latitude measurement
	 * @param north true iff the latitude measurement is north
	 * @param longitude longitude measurement
	 * @param west true iff the longitude measurement is west
	 * @param qualityValue value of the signal quality
	 * @param numSatellites number of GPS satellites connected
	 * @param horizontalDilutionOfPrecision horizontal dilation of precision
	 * @param antennaAltitude altitude of the antenna
	 */
	public GpsMeasurement(Date messageTimestamp, Date gpsTimestamp, double latitude, boolean north, double longitude,
			boolean west, int qualityValue, int numSatellites, double horizontalDilutionOfPrecision, double antennaAltitude) {
		this.timestamp = messageTimestamp;
		this.gpsTimestamp = gpsTimestamp;
		this.latitude = latitude;
		this.north = north;
		this.longitude = longitude;
		this.west = west;
		this.qualityValue = qualityValue;
		this.numSatellites = numSatellites;
		this.horizontalDilutionOfPrecision = horizontalDilutionOfPrecision;
		this.antennaAltitude = antennaAltitude;
	}
	
	/**
	 * Returns the latitude of the {@link GpsMeasurement}
	 * @return the latitude of the {@link GpsMeasurement}
	 */
	public double getLatitude() {
		return latitude;
	}
	
	/**
	 * Returns the longitude of the {@link GpsMeasurement}
	 * @return the longitude of the {@link GpsMeasurement}
	 */
	public double getLongitude() {
		return longitude;
	}
	
	/**
	 * Returns the west value of the {@link GpsMeasurement}
	 * @return the west value of the {@link GpsMeasurement}
	 */
	public boolean getWest() {
		return west;
	}
	
	/**
	 * Returns the north value of the {@link GpsMeasurement}
	 * @return the north value of the {@link GpsMeasurement}
	 */
	public boolean getNorth() {
		return north;
	}
	
	/**{@inheritDoc}*/
	@Override
	public String toLogString() {
		String s = super.formatDate(timestamp);
		
		s += ',' + super.formatDate(gpsTimestamp);
		
		s += ',' + Double.toString(latitude);
		if (north) s += ",N";
		else s += ",S";
		
		s += ',' + Double.toString(longitude);
		if (west) s += ",W";
		else s += ",E";
	
		s += "," + Integer.toString(qualityValue);
		s += "," + Integer.toString(numSatellites);
		s += "," + Double.toString(horizontalDilutionOfPrecision);
		s += "," + Double.toString(antennaAltitude);
		s += "," + Double.toString(rawGPSLat);
		s += "," + Double.toString(rawGPSLong);
		
		return s;
	}

	/**{@inheritDoc}*/
	@Override
	public Message fromLogString(String str) {
		String[] ar = str.split(",");

		Date messageTimestamp = super.tryToParseDate(ar[1]);
		Date gpsTimestamp = super.tryToParseDate(ar[0]);
		if(messageTimestamp == null || gpsTimestamp == null) {
			return null;
		}
		double latitude = Double.parseDouble(ar[1]);
		boolean north = ar[2].equalsIgnoreCase("N");
		double longitude = Double.parseDouble(ar[3]);
		boolean west = ar[3].equalsIgnoreCase("W");
		int qualityValue = Integer.parseInt(ar[4]);
		int numSatellites = Integer.parseInt(ar[5]);
		double horizontalDiluation = Double.parseDouble(ar[6]);
		double antennaAltitude = Double.parseDouble(ar[7]);
		return new GpsMeasurement(messageTimestamp, gpsTimestamp, latitude, north, longitude,
				west, qualityValue, numSatellites, horizontalDiluation, antennaAltitude);
	}
}