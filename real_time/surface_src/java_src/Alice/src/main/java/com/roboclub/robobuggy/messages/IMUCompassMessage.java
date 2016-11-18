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

    private int imuID;

    /**
     * Constructs a new {@link IMUCompassMessage} at time now with imuID = -1
     *
     * @param compassHeading compassHeading reading from the IMU
     */
    public IMUCompassMessage(double compassHeading) {
        this.timestamp = new Date().getTime();
        this.compassHeading = compassHeading;
        this.imuID = -1;
    }

    /**
     * Constructs a new {@link IMUCompassMessage} at time now
     *
     * @param compassHeading compassHeading reading from the IMU
     * @param imuID ID for the IMU
     */
    public IMUCompassMessage(double compassHeading, int imuID) {
        this.timestamp = new Date().getTime();
        this.compassHeading = compassHeading;
        this.imuID = imuID;
    }

    /**
     * Returns the compassHeading value of the {@link IMUCompassMessage}
     *
     * @return the compassHeading value of the {@link IMUCompassMessage}
     */
    public double getCompassHeading() {
        return this.compassHeading;
    }


    /**
     * Returns the ID of the IMU for the {@link IMUCompassMessage}
     *
     * @return the ID of the IMU for the {@link IMUCompassMessage}
     */
    public int getImuID() { return this.imuID; }
}
