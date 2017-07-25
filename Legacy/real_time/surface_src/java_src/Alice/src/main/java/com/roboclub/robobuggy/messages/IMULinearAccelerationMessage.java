package com.roboclub.robobuggy.messages;

import java.util.Date;

/**
 * Message for passing IMU linear acceleration measurements within BuggyROS
 *
 */
public class IMULinearAccelerationMessage extends BaseMessage {

    public static final String VERSION_ID = "LinearAccelerationV0.0";

    private double x;
    private double y;
    private double z;

    /**
     * Constructs a new {@link IMULinearAccelerationMessage} at time now
     *
     * @param x x acceleration value
     * @param y y acceleration value
     * @param z z acceleration value
     */
    public IMULinearAccelerationMessage(double x, double y, double z) {
        this.timestamp = new Date().getTime();
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

}
