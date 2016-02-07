package com.roboclub.robobuggy.messages;

import java.awt.image.BufferedImage;

import com.roboclub.robobuggy.ros.Message;

/**
 * 
 * @author Trevor Decker
 * a message for sending images between nodes 
 */
public class ImageMessage extends BaseMessage {
	// transient is used for serialization
	private transient BufferedImage thisImage;
	private int frameNumber;

	public static final String VERSION_ID = "camera_image_v0.0";
	
	/**
	 * Constructor for the image message
	 * @param newImage the image to transmit
	 */
	public ImageMessage(BufferedImage newImage, int frameNumber){
		thisImage = newImage;
		this.frameNumber= frameNumber;
	}
	
	@Override
	/**
	 * @return string this message as a log string  
	 */
	public String toLogString() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	/**
	 * @param String str this message created from a log String
	 */
	public Message fromLogString(String str) {
		// TODO Auto-generated method stub
		return null;
	}

	public int getFrameNumber(){
		return frameNumber;
	}
	
	/**
	 * 
	 * @return the image that this message is encoding as a bufferedImage
	 */
	public BufferedImage getImage() {
		return thisImage;
	}

}
