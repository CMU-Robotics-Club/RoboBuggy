package com.roboclub.robobuggy.messages;

import com.roboclub.robobuggy.ros.Message;

import java.util.Date;

/**
 * Message for passing encoder measurements over BuggyROS
 * @author ?
 *
 * @version 0.5
 * 
 *          CHANGELOG: NONE
 * 
 *          DESCRIPTION: TODO
 */
public class EncoderMeasurement extends BaseMessage {

	public static final String VERSION_ID = "encoderV0.0";

	private final double distance;
	private final double velocity;
	private final double dataWord;
	private final double accel;

	/**
	 * Construct a new {@link EncoderMeasurement} at time now
	 * @param distance the current distance value from the encoder
	 * @param velocity the current velocity value from the encoder
	 */
	public EncoderMeasurement(double distance, double velocity) {
		this.distance = distance;
		this.velocity = velocity;
		this.timestamp = new Date().getTime();
		this.dataWord = 0;
		this.accel = 0;
	}
	
	/**
	 * Construct a new {@link EncoderMeasurement}
	 * @param timestamp {@link Date} representing the time of the message
	 * @param distance the current distance value from the encoder
	 * @param velocity the current velocity value from the encoder
	 */
	public EncoderMeasurement(Date timestamp, double distance, double velocity) {
		this.timestamp = new Date(timestamp.getTime()).getTime();
		this.distance = distance;
		this.velocity = velocity;
		this.dataWord = 0;
		this.accel = 0;
	}
	
	/**
	 * Construct a new {@link EncoderMeasurement}
	 * @param timestamp {@link Date} representing the time of the message
	 * @param dataWord the exact value passed in from the encoder
	 * @param distance the current distance value from the encoder
	 * @param velocity the current velocity value from the encoder
	 * @param accel the current acceleration value from the encoder
	 */
	public EncoderMeasurement(Date timestamp, double dataWord, double distance,
			double velocity, double accel) {
		this.timestamp = new Date(timestamp.getTime()).getTime();
		this.dataWord = dataWord;
		this.distance = distance;
		this.velocity = velocity;
		this.accel = accel;		
	}
	
	/**
	 * Returns the distance from the {@link EncoderMeasurement}
	 * @return the distance from the {@link EncoderMeasurement}
	 */
	public double getDistance() {
		return distance;
	}
	
	/**
	 * Returns the velocity from the {@link EncoderMeasurement}
	 * @return the velocity from the {@link EncoderMeasurement}
	 */
	public double getVelocity() {
		return velocity;
	}
	
	/**
	 * Returns the dataWord from the {@link EncoderMeasurement}
	 * @return the dataWord from the {@link EncoderMeasurement}
	 */
	public double getDataWord() {
		return dataWord;
	}
	
	/**
	 * Returns the acceleration from the {@link EncoderMeasurement}
	 * @return the acceleration from the {@link EncoderMeasurement}
	 */
	public double getAcceleration() {
		return accel;
	}

	/**{@inheritDoc}*/
	@Override
	public String toLogString() {
		return formatDate(timestamp) + ','
				+ Double.toString(dataWord) + ','
				+ Double.toString(distance) + ',' 
				+ Double.toString(velocity) + ','
				+ Double.toString(accel);
	}

	/**{@inheritDoc}*/
	@Override
	public Message fromLogString(String str) {
		String[] ar = str.split(",");
		Date d = tryToParseDate(ar[0]);
		double distance = Double.parseDouble(ar[1]);
		double velocity = Double.parseDouble(ar[2]);
		double dataWord = Double.parseDouble(ar[3]);
		double accel = Double.parseDouble(ar[4]);
		//TODO calculate acceleration
		return new EncoderMeasurement(d, distance, velocity, dataWord, accel);
	}
}