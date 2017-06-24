package com.roboclub.robobuggy.messages;

import com.google.gson.JsonObject;
import com.roboclub.robobuggy.nodes.sensors.INodeStatus;

/**
 * Created by vivaanbahl on 2/7/16.
 */
public class NodeStatusMessage extends BaseMessage {

    public static final String VERSION_ID = "buggynode_state_v0.0";

    private Class node;
    private INodeStatus message;
    private JsonObject params;


    /**
     * @param node    the node reporting the status
     * @param message the status of the node
     * @param params  any parameters they want to pass along
     */
    public NodeStatusMessage(Class node, INodeStatus message, JsonObject params) {
        this.node = node;
        this.message = message;
        this.params = params;
    }

    /**
     * @return the node reporting status
     */
    public Class getNode() {
        return node;
    }

    /**
     * @return the status of the node
     */
    public INodeStatus getMessage() {
        return message;
    }

    /**
     * @return the paramters of the status
     */
    public JsonObject getParams() {
        return params;
    }

}
