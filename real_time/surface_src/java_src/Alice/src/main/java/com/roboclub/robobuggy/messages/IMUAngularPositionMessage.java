package com.roboclub.robobuggy.messages;

import java.util.Date;

/**
 * Message for passing IMU angular positions measurements within BuggyROS
 *
 * @author Sean Buckley
 * @version 0.5
 *          <p>
 *          CHANGELOG: NONE
 *          <p>
 *          DESCRIPTION: TODO
 */
public class IMUAngularPositionMessage extends BaseMessage {

    public static final String VERSION_ID = "imuAngularPositionV0.0";

    private int IMU;
    private double[][] rot;

    /**
     * Constructs a new {@link IMUAngularPositionMessage} at time now
     *
     * @param rotMat The matrix containing all values
     */
    public IMUAngularPositionMessage(double[][] rotMat, int imuID) {
        this.timestamp = new Date().getTime();
        this.rot = rotMat.clone();
        this.IMU = imuID;
    }

    /**
     * Returns the rot value of the {@link IMUAngularPositionMessage}
     *
     * @return the rot value of the {@link IMUAngularPositionMessage}
     */
    public double[][] getRot() {
        return this.rot.clone();
    }

    /**
     * Returns the ID of the IMU for the {@link IMUAngularPositionMessage}
     *
     * @return the ID of the IMU for the {@link IMUAngularPositionMessage}
     */
    public int getIMU() { return this.IMU; }
}
