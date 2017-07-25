package com.roboclub.robobuggy.nodes.sensors;

import com.roboclub.robobuggy.serial.RBSerialMessage;

/**
 * Class used to represent RBSM steering messages
 */
public class RBSMSteeringMessage {
    private static final byte HEADER = RBSerialMessage.getHeaderByte("RBSM_MID_MEGA_STEER_COMMAND");
    private static final byte FOOTER = RBSerialMessage.getHeaderByte("FOOTER");

    private int angle;

    /**
     * Create a new {@link RBSMSteeringMessage} object to send to the Arduino
     *
     * @param angle desired commanded angle in degrees*100
     */
    public RBSMSteeringMessage(int angle) {
        this.angle = angle;
    }

    /**
     * Returns a byte array of the {@link RBSMSteeringMessage} to send to the Arduino
     *
     * @return a byte array of the {@link RBSMSteeringMessage} to send to the Arduino
     */
    public byte[] getMessageBytes() {
        byte[] bytes = new byte[6];
        bytes[0] = HEADER;
        bytes[1] = (byte) ((angle >> 24) & 0xFF);
        bytes[2] = (byte) ((angle >> 16) & 0xFF);
        bytes[3] = (byte) ((angle >> 8) & 0xFF);
        bytes[4] = (byte) ((angle) & 0xFF);
        bytes[5] = FOOTER;
        return bytes;
    }
}
