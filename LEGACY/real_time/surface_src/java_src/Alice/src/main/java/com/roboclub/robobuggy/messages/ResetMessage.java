package com.roboclub.robobuggy.messages;

import java.util.Date;

/**
 * Message for sending a reset command over BuggyROS
 */
public class ResetMessage extends BaseMessage {
    public static final String VERSION_ID = "reset";

    /**
     * Constructs a new {@link ResetMessage}
     */
    public ResetMessage() {
        this.timestamp = new Date().getTime();
    }

}
