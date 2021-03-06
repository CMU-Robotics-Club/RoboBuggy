package com.roboclub.robobuggy.messages;

import java.util.Date;

/**
 * Message for passing IMU compass measurements within BuggyROS, in degrees from 0 - 359.9
 */
public class IMUCompassMessage extends BaseMessage {

    public static final String VERSION_ID = "imu_compass_headingV0.1";

    /**
     * a double from 0.0 to 359.9
     */
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
