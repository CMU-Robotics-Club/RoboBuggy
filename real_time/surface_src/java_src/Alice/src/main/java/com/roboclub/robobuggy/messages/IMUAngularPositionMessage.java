package com.roboclub.robobuggy.messages;

import java.util.Date;

/**
 * Message for passing IMU angular positions measurements within BuggyROS
 */
public class IMUAngularPositionMessage extends BaseMessage {

    public static final String VERSION_ID = "imuAngularPositionV0.0";

    private double[][] rot;

    /**
     * Constructs a new {@link IMUAngularPositionMessage} at time now
     *
     * @param rotMat The matrix containing all values
     */
    public IMUAngularPositionMessage(double[][] rotMat) {
        this.timestamp = new Date().getTime();
        this.rot = rotMat.clone();
    }

    /**
     * Returns the rot value of the {@link IMUAngularPositionMessage}
     *
     * @return the rot value of the {@link IMUAngularPositionMessage}
     */
    public double[][] getRot() {
        return this.rot.clone();
    }

}
