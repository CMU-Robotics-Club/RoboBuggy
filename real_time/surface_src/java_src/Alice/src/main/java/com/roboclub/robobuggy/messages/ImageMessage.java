package com.roboclub.robobuggy.messages;

import java.awt.image.BufferedImage;

import com.roboclub.robobuggy.ros.Message;

public class ImageMessage implements Message{
	private BufferedImage thisImage;
	
	
	public ImageMessage(BufferedImage newImage){
		thisImage = newImage;
	}
	
	@Override
	public String toLogString() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Message fromLogString(String str) {
		// TODO Auto-generated method stub
		return null;
	}

	public BufferedImage getImage() {
		return thisImage;
	}

}
