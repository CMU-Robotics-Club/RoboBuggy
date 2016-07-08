package com.roboclub.robobuggy.serial;

/**
 * Class used to represent a message pair of number of bytes and
 * {@link RBSerialMessage}
 */
public class RBPair {
    private int numBytesRead;
    private RBSerialMessage rbMessage;

    /**
     * Construct a new {@link RBPair}
     *
     * @param numBytesRead number of bytes read
     * @param newMessage   {@link RBSerialMessage} read
     */
    public RBPair(int numBytesRead, RBSerialMessage newMessage) {
        this.numBytesRead = numBytesRead;
        this.rbMessage = newMessage;
    }

    /**
     * Returns the number of bytes read
     *
     * @return the number of bytes read
     */
    public int getNumberOfBytesRead() {
        return numBytesRead;
    }

    /**
     * Returns the {@link RBSerialMessage} read
     *
     * @return the {@link RBSerialMessage} read
     */
    public RBSerialMessage getMessage() {
        return rbMessage;
    }

}
