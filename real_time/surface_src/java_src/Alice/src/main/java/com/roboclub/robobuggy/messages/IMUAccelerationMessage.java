package com.roboclub.robobuggy.messages;

/**
 * Created by vivaanbahl on 10/23/16.
 */
public class IMUAccelerationMessage extends BaseMessage {

    public static final String VERSION_ID = "imu_accelV0.1";

    private double xAccel;
    private double yAccel;
    private double zAccel;

    /**
     * Constructor for a new Acceleration message
     *
     * @param xAccel x acceleration
     * @param yAccel y acceleration
     * @param zAccel z acceleration
     */
    public IMUAccelerationMessage(double xAccel, double yAccel, double zAccel) {
        this.xAccel = xAccel;
        this.yAccel = yAccel;
        this.zAccel = zAccel;
    }

    /**
     * @return the x acceleration
     */
    public double getXAccel() {
        return xAccel;
    }

    /**
     * @return the y acceleration
     */
    public double getYAccel() {
        return yAccel;
    }

    /**
     * @return the z acceleration
     */
    public double getZAccel() {
        return zAccel;
    }
}
