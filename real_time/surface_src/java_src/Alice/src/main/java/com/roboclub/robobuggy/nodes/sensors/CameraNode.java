package com.roboclub.robobuggy.nodes.sensors;

import com.github.sarxos.webcam.Webcam;
import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.messages.ImageMessage;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyBaseNode;
import com.roboclub.robobuggy.nodes.baseNodes.PeriodicNode;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;

import java.awt.image.BufferedImage;
import java.util.List;

/**
 * 
 * @author Trevor Decker
 * 
 * A software driver for listening to a camera and publishing images from 
 * it to a message channel 
 * 
 */
public class CameraNode extends PeriodicNode{
	private Webcam webcam;
	private Publisher imagePublisher;
	private Publisher imageFramePublisher;
	private int count = 0;
	
	/**
	 *  @param channel The channel to publish messages on
	 * @param period  How often new images should be pulled
	 */
	public CameraNode(NodeChannel channel, int period) {
		super(new BuggyBaseNode(channel),period);


		//setup the webcam
		List<Webcam> webcams = Webcam.getWebcams();
		this.webcam = null;

		//TODO figure out a better way to select
		for (Webcam webcam : webcams) {
			if (webcam.getName().contains("Logitech")) {
				this.webcam = webcam;
				break;
			}
		}

		if (this.webcam == null) {
			new RobobuggyLogicNotification("Couldn't find Logitech webcam!", RobobuggyMessageLevel.EXCEPTION);
			return;
		}

		webcam.open();

		//setup image publisher
		imagePublisher = new Publisher(channel.getMsgPath());
	}



	@Override
	protected boolean startDecoratorNode() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	protected boolean shutdownDecoratorNode() {
		webcam.close();
		return false;
	}
	
	@Override
	protected void update() {
		if(webcam != null && imagePublisher != null){
			if (webcam.isOpen()) {
				BufferedImage mostRecentImage = webcam.getImage();
				imagePublisher.publish(new ImageMessage(mostRecentImage, count));
				count = count + 1;
			}
			else {
				if(!webcam.open()) {
					new RobobuggyLogicNotification("Webcam was closed and couldn't be reopened!", RobobuggyMessageLevel.EXCEPTION);
				}
				else {
					new RobobuggyLogicNotification("Webcam was closed but successfully reopened!", RobobuggyMessageLevel.WARNING);
				}
			}
		}

	}

}
