package com.roboclub.robobuggy.ui;

import java.awt.Color;
import java.awt.GridLayout;

import javax.swing.BorderFactory;
import javax.swing.JPanel;

import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.ros.Subscriber;

public class GraphPanel extends RoboBuggyGUIContainer {
	private static final long serialVersionUID = -5453262887347328140L;

	private AngleGraph steeringGraph;
	private AngleGraph rollGraph;
	private AngleGraph pitchGraph;
	private AngleGraph yawGraph;
	
	public GraphPanel() {
		
		steeringGraph = new AngleGraph("STEERING");
		rollGraph = new AngleGraph("ROLL");
		pitchGraph = new AngleGraph("PITCH");
		yawGraph = new AngleGraph("YAW");
		
		//add the graphs to the container
		this.addComponet(steeringGraph, 0, 0, .25, 1.0);
		this.addComponet(rollGraph, .25, 0, .25, 1.0);
		this.addComponet(pitchGraph, .50, 0, .25, 1.0);
		this.addComponet(yawGraph, .75, 0, .25, 1.0);
		
		// Subscriber for drive control updates
		new Subscriber(SensorChannel.STEERING.getMsgPath(), new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				steeringGraph.updateGraph(((SteeringMeasurement)m).angle);
			    Gui.getInstance().fixPaint();
			}
		});
		
		// Subscriber for Imu updates
		new Subscriber(SensorChannel.IMU.getMsgPath(), new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
			    Gui.getInstance().fixPaint();
				// TODO handle imu updates for graphs
			}
		});
	}
}
