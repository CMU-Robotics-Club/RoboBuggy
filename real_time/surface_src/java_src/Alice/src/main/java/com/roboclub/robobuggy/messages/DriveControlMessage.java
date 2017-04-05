package com.roboclub.robobuggy.messages;

import java.util.Date;

/**
 * Message for passing the desired commanded value of the steering
 */
public class DriveControlMessage extends BaseMessage {

    public static final String VERSION_ID = "drive_control_message";

    /**
     *  Angle is supposed to be in radians
     */
    private final double angle;
    private GpsMeasurement currentWaypoint;

    /**
     * Construct a new DriveControlMessage
     *
     * @param timestamp {@link Date} representing the creation time
     * @param angle     the commanded angle of the front wheel
     */
    public DriveControlMessage(Date timestamp, double angle) {
        this.angle = angle;
        this.timestamp = new Date(timestamp.getTime()).getTime();
    }

    public DriveControlMessage(Date timestamp, double angle, GpsMeasurement currentWaypoint) {
        this.angle = angle;
        this.timestamp = new Date(timestamp.getTime()).getTime();
        this.currentWaypoint = currentWaypoint;
    }

    /**
     * Returns the commanded angle of the steering as a double (in degrees)
     *
     * @return the commanded angle of the steering as a double (in degrees)
     */
    public double getAngleDouble() {
        return angle;
    }

}
