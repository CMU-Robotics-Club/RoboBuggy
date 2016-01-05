package com.roboclub.robobuggy.ui;

import java.awt.Color;
import java.awt.GridLayout;
import javax.swing.BorderFactory;
import javax.swing.JPanel;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Subscriber;

public class GraphPanel extends JPanel {
	private static final long serialVersionUID = -5453262887347328140L;

	private AngleGraph steering;
	private AngleGraph roll;
	private AngleGraph pitch;
	private AngleGraph yaw;
	
	public GraphPanel() {
		this.setBorder(BorderFactory.createLineBorder(Color.black));
		this.setLayout(new GridLayout(1,4));
		
		steering = new AngleGraph("STEERING");
		roll = new AngleGraph("ROLL");
		pitch = new AngleGraph("PITCH");
		yaw = new AngleGraph("YAW");
		
		this.add(steering);
		this.add(roll);
		this.add(pitch);
		this.add(yaw);
		
		// Subscriber for drive control updates
		new Subscriber(NodeChannel.STEERING.getMsgPath(), new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				steering.updateGraph(((SteeringMeasurement)m).angle);
			}
		});
		
		// Subscriber for Imu updates
		new Subscriber(NodeChannel.IMU.getMsgPath(), new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				// TODO handle imu updates for graphs
			}
		});
	}
}
