package com.roboclub.robobuggy.messages;

import java.util.Date;

/**
 * Message for passing IMU compassHeading measurements within BuggyROS
 *
 * @author ?
 * @version 0.5
 *          <p>
 *          CHANGELOG: NONE
 *          <p>
 *          DESCRIPTION: TODO
 */
public class IMUCompassMessage extends BaseMessage {

    public static final String VERSION_ID = "temperatureV0.1";

    private double compassHeading;

    /**
     * Constructs a new {@link IMUCompassMessage} at time now
     *
     * @param compassHeading compassHeading reading from the IMU
     */
    public IMUCompassMessage(double compassHeading) {
        this.timestamp = new Date().getTime();
        this.compassHeading = compassHeading;
    }

    /**
     * Returns the compassHeading value of the {@link IMUCompassMessage}
     *
     * @return the compassHeading value of the {@link IMUCompassMessage}
     */
    public double getCompassHeading() {
        return this.compassHeading;
    }

}
