package com.roboclub.robobuggy.messages;

import com.roboclub.robobuggy.messaging_autogen.RobobuggyMessagingProto.RobobuggyMessage;

/**
 * Created by vivaanbahl on 10/7/16.
 */
public class RobobuggyProtoWrapper extends BaseMessage {
    private RobobuggyMessage protoMessage;

    /**
     * Creates a new message wrapper for a protobuf message
     * @param sensor the sensor the message is coming from
     * @param sensorSubtype the subtype of the sensor (imu temperature vs imu magnetometer)
     * @param messageOrigin The class the message originated from
     */
    public RobobuggyProtoWrapper(String sensor, String sensorSubtype, String messageOrigin) {
        protoMessage = RobobuggyMessage.newBuilder()
                            .setSensor(sensor)
                            .setSensorSubtype(sensorSubtype)
                            .setMessageOrigin(messageOrigin)
                .build();
    }

    /**
     * Gives the protobuf message for any data collection
     * @return the internal protobuf message
     */
    public RobobuggyMessage getProtoMessage() {
        return protoMessage;
    }

}
