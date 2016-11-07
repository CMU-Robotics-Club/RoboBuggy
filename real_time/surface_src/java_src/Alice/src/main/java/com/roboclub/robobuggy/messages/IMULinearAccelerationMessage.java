package com.roboclub.robobuggy.messages;

import java.util.Date;

/**
 * Message for passing IMU linear acceleration measurements within BuggyROS
 *
 * @author Sean Buckley
 * @version 0.5
 *          <p>
 *          CHANGELOG: NONE
 *          <p>
 *          DESCRIPTION: TODO
 */
public class IMULinearAccelerationMessage extends BaseMessage {

    public static final String VERSION_ID = "LinearAccelerationV0.0";

    int IMU;

    private double x;
    private double y;
    private double z;

    /**
     * Constructs a new {@link IMULinearAccelerationMessage} at time now
     *
     * @param x x acceleration value
     * @param y y acceleration value
     * @param z z acceleration value
     * @param imuID the ID of the IMU that generated the data
     */
    public IMULinearAccelerationMessage(double x, double y, double z, int imuID) {
        this.timestamp = new Date().getTime();
        this.IMU = imuID;
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /**
     * Returns the x value of the {@link IMULinearAccelerationMessage}
     *
     * @return the x value of the {@link IMULinearAccelerationMessage}
     */
    public double getX() {
        return x;
    }

    /**
     * Returns the y value of the {@link IMULinearAccelerationMessage}
     *
     * @return the y value of the {@link IMULinearAccelerationMessage}
     */
    public double getY() {
        return y;
    }

    /**
     * Returns the z value of the {@link IMULinearAccelerationMessage}
     *
     * @return the z value of the {@link IMULinearAccelerationMessage}
     */
    public double getZ() {
        return z;
    }

    /**
     * Returns the ID of the IMU for the {@link IMULinearAccelerationMessage}
     *
     * @return the ID of the IMU for the {@link IMULinearAccelerationMessage}
     */
    public int getIMU() { return this.IMU; }
}
