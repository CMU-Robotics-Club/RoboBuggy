package com.roboclub.robobuggy.nodes.sensors;

import java.awt.image.BufferedImage;
import java.util.List;



import com.github.sarxos.webcam.Webcam;
import com.orsoncharts.util.json.JSONObject;
import com.roboclub.robobuggy.messages.ImageMessage;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyBaseNode;
import com.roboclub.robobuggy.nodes.baseNodes.PeriodicNode;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;

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
	private int count = 0;
	
	/**
	 * 
	 * @param channel The channel to publish messages on 
	 * @param period  How often new images should be pulled 
	 */
	public CameraNode(NodeChannel channel, int period) {
		super(new BuggyBaseNode(channel),period);
		
		
		//setup the webcam
		List<Webcam> webcams = Webcam.getWebcams();
		webcam = webcams.get(0); //todo some selection logic
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
			//TODO: check to see if the webcam is closed if so try to reconnect 
			BufferedImage mostRecentImage = webcam.getImage();
			imagePublisher.publish(new ImageMessage(mostRecentImage));
		}

	}


	/**
	 * produces a jsonobject from a log message
	 * @param message the log message that the jsonobject is going to be produced from 
	 * @return the Jsonobject 
	 */
	public static JSONObject toJObject(String message) {
		JSONObject data = new JSONObject();
		JSONObject params = new JSONObject();
		String[] messageData = message.split(",");
		data.put("timestamp", messageData[1]);
		data.put("params", params);
		return data;
	}

}
