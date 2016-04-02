package com.roboclub.robobuggy.nodes.sensors;

import com.roboclub.robobuggy.ros.Message;

/**
 * a class that helps keep track of where a message came from, incase multiple publishers use the same type of message
 */
class TraceableMessage {
    private Message message;
    private String topic;

    public TraceableMessage(Message m, String topic) {
        this.message = m;
        this.topic = topic;
    }

    public Message getMessage() {
        return message;
    }

    public void setMessage(Message message) {
        this.message = message;
    }

    public String getTopic() {
        return topic;
    }

    public void setTopic(String topic) {
        this.topic = topic;
    }
}