package com.roboclub.robobuggy.messages;

/**
 * Reports the IMU's inclination, in pitch-roll-yaw format
 */
public class IMUInclinationMessage extends BaseMessage {

    public static final String VERSION_ID = "imu_inclinationV0.1";

    private double xInclination;
    private double yInclination;
    private double zInclination;

    private int imuID;

    /**
     * Constructs a new Inclination Message with imuID = -1
     * @param xInclination x inclination (pitch)
     * @param yInclination y inclination (roll)
     * @param zInclination z inclination (yaw)
     */
    public IMUInclinationMessage(double xInclination, double yInclination, double zInclination) {
        this.xInclination = xInclination;
        this.yInclination = yInclination;
        this.zInclination = zInclination;
        this.imuID = -1;
    }

    /**
     * Constructs a new Inclination Message
     * @param xInclination x inclination (pitch)
     * @param yInclination y inclination (roll)
     * @param zInclination z inclination (yaw)
     * @param imuID the ID of the IMU that generated this message
     */
    public IMUInclinationMessage(double xInclination, double yInclination, double zInclination, int imuID) {
        this.xInclination = xInclination;
        this.yInclination = yInclination;
        this.zInclination = zInclination;
        this.imuID = imuID;
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

    /**
     * Returns the ID of the IMU for the {@link IMUInclinationMessage}
     *
     * @return the ID of the IMU for the {@link IMUInclinationMessage}
     */
    public int getImuID() { return this.imuID; }
}
