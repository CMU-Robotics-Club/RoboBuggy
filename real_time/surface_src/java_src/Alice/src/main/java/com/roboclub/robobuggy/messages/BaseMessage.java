package com.roboclub.robobuggy.messages;

import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.ros.Message;

import java.text.DateFormat;
import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * Abstract class used to represent the base message sent over BuggyROS
 */
public abstract class BaseMessage implements Message {
	private static final String DATE_FORMAT = "yyyy-MM-dd HH:mm:ss.SSS";
	protected long timestamp;
    	private String topicName;
	private long sequenceNumber = -1;

	/**
	 * sets the date of the message to the current time
	 */
	public BaseMessage() {
		timestamp = new Date().getTime();
	}

	/**
	 * Publishers number their messages, increasing one each time.
	 * If a message is dropped, then the subscriber will know if they keep
	 * track of the current sequence number.
	 * @return sequence number of the current message
	 */
	public long getSequenceNumber() {
		return sequenceNumber;
	}

	/**
	 * Sets the sequence number of this message. 
	 * This should only be called by Publisher; it will
	 * be overwritten if anyone else writes to it.
	 * @param sequenceNumber sequence number of the current message. Ignored if not called from publisher.java
	 */
	public void setSequenceNumber(long sequenceNumber) {
		this.sequenceNumber = sequenceNumber;
	}

	/**
	 * Creates a {@link String} representing the {@link Date}
	 * @param dt {@link Date} to format
	 * @return a {@link String} representing the {@link Date} dt
	 */
	public static String formatDate(long dt) {
		Date date = new Date(dt);
		DateFormat formatter = new SimpleDateFormat(DATE_FORMAT);
		return formatter.format(date);
	}

	/**
	 * Converts a {@link String} into a valid {@link Date} object, if possible
	 * @param maybeDate {@link String} representing the {@link Date}
	 * @return a new {@link Date} object representing maybeDate, or null if
	 *  maybeDate is an invalid format
	 */
	public static Date tryToParseDate(String maybeDate) {
		try {
			DateFormat formatter = new SimpleDateFormat(DATE_FORMAT);
			return formatter.parse(maybeDate);
		} catch (ParseException e) {
			new RobobuggyLogicNotification("could not parse date stack trace: "+ e.getMessage(), RobobuggyMessageLevel.WARNING);
			return null;
		}
	}

	/**
	 * Returns the timestamp of the message
	 * @return time that this message was instantiated
	 */
	public Date getTimestamp() {
		return new Date(timestamp);
	}

    /**
     * Sets the topic name that the message is on
     * This allows us to know which topic a message came from; very useful for serialization
     * @param topicName Topic to set the name to
     */
    public void setTopicName(String topicName) {
        this.topicName = topicName;
    }
    
    /**
     * Gets the topic name for this message
     * @return name of the current topic
     */
    public String getTopicName() {
        return this.topicName;
    }
}
