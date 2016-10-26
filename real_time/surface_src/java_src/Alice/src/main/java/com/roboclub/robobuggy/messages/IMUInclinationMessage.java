package com.roboclub.robobuggy.messages;

/**
 * Reports the IMU's inclination, in roll-pitch-yaw format
 */
public class IMUInclinationMessage extends BaseMessage {

    public static final String VERSION_ID = "imu_inclinationV0.1";

    private double xInclination;
    private double yInclination;
    private double zInclination;

    /**
     * Constructs a new Inclination Message
     * @param xInclination x inclination (pitch)
     * @param yInclination y inclination (roll)
     * @param zInclination z inclination (yaw)
     */
    public IMUInclinationMessage(double xInclination, double yInclination, double zInclination) {
        this.xInclination = xInclination;
        this.yInclination = yInclination;
        this.zInclination = zInclination;
    }

    /**
     * @return x inclination
     */
    public double getXInclination() {
        return xInclination;
    }

    /**
     * @return y inclination
     */
    public double getYInclination() {
        return yInclination;
    }

    /**
     * @return z inclination
     */
    public double getZInclination() {
        return zInclination;
    }
}
