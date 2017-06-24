package com.roboclub.robobuggy.messages;

import java.util.Date;

/**
 * Message for passing the desired commanded value of the brakes
 */
public class BrakeControlMessage extends BaseMessage {

    public static final String VERSION_ID = "brake_control_message";

    private final boolean brakeEngaged;

    /**
     * Construct a new BrakeControlMessage
     *
     * @param timestamp     {@link Date} representing the creation time
     * @param brakeEngagged whether or not the brakes are deployed
     */
    public BrakeControlMessage(Date timestamp, boolean brakeEngagged) {
        this.brakeEngaged = brakeEngagged;
        this.timestamp = new Date(timestamp.getTime()).getTime();
    }

    /**
     * Returns true iff the brake is commanded to be engaged
     *
     * @return true iff the brake is commanded to be engaged
     */
    public boolean isBrakeEngaged() {
        return brakeEngaged;
    }

}
