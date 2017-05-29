package com.roboclub.robobuggy.messages;

import java.awt.image.BufferedImage;

/**
 * @author Trevor Decker
 *         a message for sending images between nodes
 */
public class ImageMessage extends BaseMessage {
    // transient is used for serialization
    private transient BufferedImage thisImage;
    private int frameNumber;

    public static final String VERSION_ID = "camera_image_v0.0";

    /**
     * Constructor for the image message
     *
     * @param newImage    the image to transmit;
     * @param frameNumber the frame number
     */
    public ImageMessage(BufferedImage newImage, int frameNumber) {
        thisImage = newImage;
        this.frameNumber = frameNumber;
    }

    /**
     * @return the frame number of the image
     */
    public int getFrameNumber() {
        return frameNumber;
    }

    /**
     * @return the image that this message is encoding as a bufferedImage
     */
    public BufferedImage getImage() {
        return thisImage;
    }

}
