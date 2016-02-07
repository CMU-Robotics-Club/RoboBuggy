package com.roboclub.robobuggy.messages;

import com.roboclub.robobuggy.ros.Message;

/**
 * Created by vivaanbahl on 2/7/16.
 */
public class NodeStatusMessage extends BaseMessage {

    public static final String VERSION_ID = "buggynode_state_v0.0";

    private Class node;
    private String message;


    public NodeStatusMessage(Class node, String message) {
        this.node = node;
        this.message = message;
    }

    public Class getNode() {
        return node;
    }

    public String getMessage() {
        return message;
    }

    @Override
    public String toLogString() {
        return null;
    }

    @Override
    public Message fromLogString(String str) {
        return null;
    }
}
