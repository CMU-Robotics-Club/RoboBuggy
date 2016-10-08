package com.roboclub.robobuggy.messages;

import com.roboclub.robobuggy.messaging_autogen.RobobuggyMessagingProto.RobobuggyMessage;
import com.roboclub.robobuggy.messaging_autogen.RobobuggyMessagingProto.RobobuggyMessage.RobobuggyMessageMetadata;
import com.roboclub.robobuggy.messaging_autogen.RobobuggyMessagingProto.RobobuggyMessage.RobobuggyMessageParameterSet;
import com.roboclub.robobuggy.messaging_autogen.RobobuggyMessagingProto.RobobuggyMessage.RobobuggySensors;

/**
 * Created by vivaanbahl on 10/7/16.
 * A wrapper class for the protobuf messages
 */
public class RobobuggyProtoWrapper extends BaseMessage {
    private RobobuggyMessage.Builder protoMessageBuilder;

    /**
     * Creates a new message wrapper for a protobuf message
     * @param sensor the sensor the message is coming from
     * @param sensorSubtype the subtype of the sensor (imu temperature vs imu magnetometer)
     */
    public RobobuggyProtoWrapper(RobobuggySensors sensor, String sensorSubtype) {
        RobobuggyMessageMetadata metadata = RobobuggyMessageMetadata.newBuilder()
                .setSensor(sensor)
                .setSensorSubtype(sensorSubtype)
                .build();

        // start with an empty parameter set
        protoMessageBuilder = RobobuggyMessage.newBuilder()
                .setMetadata(metadata);
    }

    /**
     * Adds the key-int pair to the message
     * @param key the key
     * @param value the value of type int
     */
    public void addParameter(String key, int value) {
        addParameter(key, value, null, null, null);
    }

    /**
     * Adds the key-bool pair to the message
     * @param key the key
     * @param value the value of type bool
     */
    public void addParameter(String key, boolean value) {
        addParameter(key, null, value, null, null);
    }

    /**
     * Adds the key-double pair to the message
     * @param key the key
     * @param value the value of type double
     */
    public void addParameter(String key, double value) {
        addParameter(key, null, null, value, null);
    }

    /**
     * Adds the key-string pair to the message
     * @param key the key
     * @param value the value of type string
     */
    public void addParameter(String key, String value) {
        addParameter(key, null, null, null, value);
    }

    /**
     * Method for adding a key-value pair for the parameters
     * @param key the key of the parameter
     * @param valueAsInt non-null if the value is of type int
     * @param valueAsBool non-null if the value is of type bool
     * @param valueAsDouble non-null if the value is of type double
     * @param valueAsString non-null if the value is of type String
     */
    private void addParameter(String key, Integer valueAsInt, Boolean valueAsBool, Double valueAsDouble, String valueAsString) {
        int numberOfExistingParams = protoMessageBuilder.getParamsCount();

        RobobuggyMessageParameterSet.Builder parametersBuilder = RobobuggyMessageParameterSet.newBuilder()
                .setKey(key);

        if (valueAsInt != null) {
            parametersBuilder.setValueAsInt(valueAsInt);
        }
        else if (valueAsBool != null) {
            parametersBuilder.setValueAsBool(valueAsBool);
        }
        else if (valueAsDouble != null) {
            parametersBuilder.setValueAsDouble(valueAsDouble);
        }
        else if (valueAsString != null) {
            parametersBuilder.setValueAsString(valueAsString);
        }

        protoMessageBuilder.setParams(numberOfExistingParams, parametersBuilder);
    }


    /**
     * Gives the protobuf message for any data collection
     * @return the internal protobuf message
     */
    public RobobuggyMessage getProtoMessage() {
        return protoMessageBuilder.build();
    }

}
