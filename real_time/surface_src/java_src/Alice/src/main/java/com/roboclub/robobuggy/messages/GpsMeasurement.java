package com.roboclub.robobuggy.messages;

import java.util.Date;

import com.roboclub.robobuggy.main.config;
import com.roboclub.robobuggy.ros.Message;

/**
 * @author Matt Sebek (msebek)
 *
 * @version 0.5
 * 
 *          CHANGELOG: NONE
 * 
 *          DESCRIPTION: TODO
 */

// gpsTimestamp is the UTC time in hours for the given position.
// See http://hemispheregnss.com/gpsreference/GPGGA.html for more information
public class GpsMeasurement extends BaseMessage implements Message {
	public static final String version_id = "gpsV0.2";

	public Date timestamp;
	public Date gpsTimestamp;
	public double latitude;
	public boolean north;
	public double longitude;
	public boolean west;
	public int quality_value;
	public int num_satellites;
	public double horizontal_dilution_of_precision;
	public double antenna_altitude;
	
	public GpsMeasurement(Date gpsTimestamp, double latitude, boolean north, double longitude, 
			boolean west, int quality_value, int num_satellites, 
			double horizontal_dulition_of_precision, double antenna_altitude) {
	
		this.timestamp = new Date();
		this.gpsTimestamp = gpsTimestamp;
		this.latitude = latitude;
		this.north = north;
		this.longitude = longitude;
		this.west = west;
		this.quality_value = quality_value;
		this.num_satellites = num_satellites;
		this.horizontal_dilution_of_precision = horizontal_dulition_of_precision;
		this.antenna_altitude = antenna_altitude;
	}

	public GpsMeasurement(Date messageTimestamp, Date gpsTimestamp, double latitude, boolean north, double longitude,
			boolean west, int quality_value, int num_satellites, double horizontal_dilution_of_precision, double antenna_altitude) {
		
		this.timestamp = messageTimestamp;
		this.gpsTimestamp = gpsTimestamp;
		this.latitude = latitude;
		this.north = north;
		this.longitude = longitude;
		this.west = west;
		this.quality_value = quality_value;
		this.num_satellites = num_satellites;
		this.horizontal_dilution_of_precision = horizontal_dilution_of_precision;
		this.antenna_altitude = antenna_altitude;
	}
	
	@Override
	public String toLogString() {
		String s = super.format_the_date(timestamp);
		
		s += ',' + super.format_the_date(gpsTimestamp);
		
		s += ',' + Double.toString(latitude);
		if (north) s += ",N";
		else s += ",S";
		
		s += ',' + Double.toString(longitude);
		if (west) s += ",W";
		else s += ",E";
	
		s += "," + Integer.toString(quality_value);
		s += "," + Integer.toString(num_satellites);
		s += "," + Double.toString(horizontal_dilution_of_precision);
		s += "," + Double.toString(antenna_altitude);
		
		return s;
	}

	@Override
	public Message fromLogString(String str) {
		String[] ar = str.split(",");

		Date messageTimestamp = super.try_to_parse_date(ar[1]);
		Date gpsTimestamp = super.try_to_parse_date(ar[0]);
		if(messageTimestamp == null || gpsTimestamp == null) {
			return null;
		}
		double latitude = Double.parseDouble(ar[1]);
		boolean north = ar[2].equalsIgnoreCase("N");
		double longitude = Double.parseDouble(ar[3]);
		boolean west = ar[3].equalsIgnoreCase("W");
		int quality_value = Integer.parseInt(ar[4]);
		int num_satellites = Integer.parseInt(ar[5]);
		double horizontal_diluation = Double.parseDouble(ar[6]);
		double antenna_altitude = Double.parseDouble(ar[7]);
		return new GpsMeasurement(messageTimestamp, gpsTimestamp, latitude, north, longitude,
				west, quality_value, num_satellites, horizontal_diluation, antenna_altitude);
	}

	@Override
	public String getCorrespondingSensor() {
		// TODO Auto-generated method stub
		return config.SENSOR_NAME_GPS;
	}
}