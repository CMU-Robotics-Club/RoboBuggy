package com.roboclub.robobuggy.messages;

import java.util.Date;

/**
 * Message for passing IMU measurements within BuggyROS
 *
 * @author ?
 * @version 0.5
 *          <p>
 *          CHANGELOG: NONE
 *          <p>
 *          DESCRIPTION: TODO
 */
@Deprecated
public class ImuMeasurement extends BaseMessage {

    public static final String VERSION_ID = "imuV0.0";


    private double yaw;
    private double pitch;
    private double roll;

    /**
     * Constructs a new {@link ImuMeasurement} at time now
     *
     * @param y yaw value
     * @param p pitch value
     * @param r roll value
     */
    public ImuMeasurement(double y, double p, double r) {
        this.timestamp = new Date().getTime();
        this.yaw = y;
        this.pitch = p;
        this.roll = r;
    }

    /**
     * Returns the yaw value of the {@link ImuMeasurement}
     *
     * @return the yaw value of the {@link ImuMeasurement}
     */
    public double getYaw() {
        return yaw;
    }

    /**
     * Returns the pitch value of the {@link ImuMeasurement}
     *
     * @return the pitch value of the {@link ImuMeasurement}
     */
    public double getPitch() {
        return pitch;
    }

    /**
     * Returns the roll value of the {@link ImuMeasurement}
     *
     * @return the roll value of the {@link ImuMeasurement}
     */
    public double getRoll() {
        return roll;
    }

}
