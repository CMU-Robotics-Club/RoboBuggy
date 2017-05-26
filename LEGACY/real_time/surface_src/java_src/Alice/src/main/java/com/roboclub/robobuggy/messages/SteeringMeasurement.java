package com.roboclub.robobuggy.messages;

import java.util.Date;

/**
 * Message for passing steering angle measurements over BuggyROS
 */
public class SteeringMeasurement extends BaseMessage {
    public static final String VERSION_ID = "steering";
    private double angle;

    /**
     * Construct a new {@link SteeringMeasurement} at time now
     *
     * @param d front wheel angle
     */
    public SteeringMeasurement(double d) {
        this.timestamp = new Date().getTime();
        this.angle = d;
    }

    /**
     * Returns the angle of the {@link SteeringMeasurement}
     *
     * @return the angle of the {@link SteeringMeasurement}
     */
    public double getAngle() {
        return angle;
    }

}
