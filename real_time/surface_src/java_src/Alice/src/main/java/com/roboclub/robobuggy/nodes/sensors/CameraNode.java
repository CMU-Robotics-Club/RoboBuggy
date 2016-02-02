package com.roboclub.robobuggy.nodes.sensors;

import java.awt.image.BufferedImage;
import java.awt.peer.RobotPeer;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

import org.jcodec.api.awt.SequenceEncoder;

import com.github.sarxos.webcam.Webcam;
import com.orsoncharts.util.json.JSONObject;
import com.roboclub.robobuggy.messages.ImageMessage;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyBaseNode;
import com.roboclub.robobuggy.nodes.baseNodes.PeriodicNode;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;

public class CameraNode extends PeriodicNode{
	private Webcam webcam;
	private SequenceEncoder videoEncoder;
	private Publisher imagePublisher;
	private int count = 0;
	
	public CameraNode(NodeChannel channel, int period) {
		super(new BuggyBaseNode(channel),period);
		
		//setup the video encoding
		//TODO move to logging node
		try {
			videoEncoder = new SequenceEncoder(new File("test.mp4"));
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		//setup the webcam
		webcam = Webcam.getDefault();
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
		try {
			videoEncoder.finish();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return false;
	}
	
	@Override
	protected void update() {
		if(webcam != null && imagePublisher != null){
			//TODO: check to see if the webcam is closed if so try to reconnect 
			BufferedImage mostRecentImage = webcam.getImage();
			imagePublisher.publish(new ImageMessage(mostRecentImage));
		/*
			try {
				if(count < 100){
					videoEncoder.encodeImage(mostRecentImage);
					count++;
					System.out.println("count:"+count);
				}else{
					videoEncoder.finish();
				}
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		*/
		}

	}



	public static JSONObject toJObject(String message) {
		JSONObject data = new JSONObject();
		JSONObject params = new JSONObject();
		String[] messageData = message.split(",");
		data.put("timestamp", messageData[1]);
		data.put("params", params);
		return data;
	}

}
