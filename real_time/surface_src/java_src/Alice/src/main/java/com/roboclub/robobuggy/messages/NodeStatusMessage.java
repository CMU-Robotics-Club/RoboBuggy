package com.roboclub.robobuggy.messages;

import com.google.gson.JsonObject;
import com.roboclub.robobuggy.nodes.sensors.INodeStatus;
import com.roboclub.robobuggy.ros.Message;

/**
 * Created by vivaanbahl on 2/7/16.
 */
public class NodeStatusMessage extends BaseMessage {

    public static final String VERSION_ID = "buggynode_state_v0.0";

    private Class node;
    private INodeStatus message;
    private JsonObject params;


    public NodeStatusMessage(Class node, INodeStatus message, JsonObject params) {
        this.node = node;
        this.message = message;
        this.params = params;
    }

    public Class getNode() {
        return node;
    }

    public INodeStatus getMessage() {
        return message;
    }

    public JsonObject getParams() {
        return params;
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
