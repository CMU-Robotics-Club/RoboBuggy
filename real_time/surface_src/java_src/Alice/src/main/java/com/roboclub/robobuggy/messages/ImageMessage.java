package com.roboclub.robobuggy.messages;

import java.awt.image.BufferedImage;

import com.roboclub.robobuggy.ros.Message;

/**
 * 
 * @author Trevor Decker
 * a message for sending images between nodes 
 */
public class ImageMessage implements Message{
	private BufferedImage thisImage;
	
	/**
	 * Constructor for the image message
	 * @param newImage the image to transmit
	 */
	public ImageMessage(BufferedImage newImage){
		thisImage = newImage;
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

	/**
	 * 
	 * @return the image that this message is encoding as a bufferedImage
	 */
	public BufferedImage getImage() {
		return thisImage;
	}

}
