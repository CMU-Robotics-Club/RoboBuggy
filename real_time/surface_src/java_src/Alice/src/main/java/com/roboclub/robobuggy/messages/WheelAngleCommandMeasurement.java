package com.roboclub.robobuggy.messages;

import java.util.Date;

/**
 * Message used to pass commanded steering angle messages over BuggyROS
 *
 * @author ?
 * @version 0.5
 *          <p>
 *          CHANGELOG: NONE
 *          <p>
 *          DESCRIPTION: TODO
 */
public class WheelAngleCommandMeasurement extends BaseMessage {
    // V0.0 had int
    // V0.1 has float. Note that round-off is hundreths of a degree.
    public static final String VERSION_ID = "autonomous_angleV0.1";

    private float angle;

    /**
     * Construct a new {@link WheelAngleCommandMeasurement} object at time now
     *
     * @param angle the commanded angle
     *              (Note that round-off is hundreths of a degree)
     */
    public WheelAngleCommandMeasurement(float angle) {
        this.angle = angle;
        this.timestamp = new Date().getTime();
    }

    /**
     * Get the angle commanded for the wheel
     *
     * @return the commanded angle
     */
    public float getAngle() {
        return angle;
    }


}