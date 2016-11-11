package com.roboclub.robobuggy.messages;

import java.util.Date;

/**
 * Message for passing IMU angular velocity measurements within BuggyROS
 */
public class IMUAngularVelocityMessage extends BaseMessage {

    public static final String VERSION_ID = "imuAngularVelocityV0.0";

    private int IMU;

    private double x;
    private double y;
    private double z;

    /**
     * Constructs a new {@link IMUAngularVelocityMessage} at time now with IMU ID 0
     *
     * @param x x value
     * @param y y value
     * @param z z value
     */
    public IMUAngularVelocityMessage(double x, double y, double z) {
        this.timestamp = new Date().getTime();
        this.IMU = 0;
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /**
     * Constructs a new {@link IMUAngularVelocityMessage} at time now
     *
     * @param x x value
     * @param y y value
     * @param z z value
     * @param imuID the ID of the IMU that generated the data
     */
    public IMUAngularVelocityMessage(double x, double y, double z, int imuID) {
        this.timestamp = new Date().getTime();
        this.IMU = imuID;
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /**
     * Returns the x value of the {@link IMUAngularVelocityMessage}
     *
     * @return the x value of the {@link IMUAngularVelocityMessage}
     */
    public double getX() {
        return x;
    }

    /**
     * Returns the y value of the {@link IMUAngularVelocityMessage}
     *
     * @return the y value of the {@link IMUAngularVelocityMessage}
     */
    public double getY() {
        return y;
    }

    /**
     * Returns the z value of the {@link IMUAngularVelocityMessage}
     *
     * @return the z value of the {@link IMUAngularVelocityMessage}
     */
    public double getZ() {
        return z;
    }

    /**
     * Returns the ID of the IMU for the {@link IMUAngularPositionMessage}
     *
     * @return the ID of the IMU for the {@link IMUAngularPositionMessage}
     */
    public int getIMU() { return this.IMU; }

}
