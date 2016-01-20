package com.roboclub.robobuggy.ui;


import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Subscriber;

/**
 * {@link RoboBuggyGUIContainer} used to represent a graph
 */
public class GraphPanel extends RoboBuggyGUIContainer {

	private static final long serialVersionUID = -5453262887347328140L;

	private AngleGraph steeringGraph;
	private AngleGraph rollGraph;
	private AngleGraph pitchGraph;
	private AngleGraph yawGraph;
	
	/**
	 * Construct a new {@link GraphPanel}
	 */
	public GraphPanel() {
		
		steeringGraph = new AngleGraph("STEERING");
		rollGraph = new AngleGraph("ROLL");
		pitchGraph = new AngleGraph("PITCH");
		yawGraph = new AngleGraph("YAW");
		
		//add the graphs to the container
		this.addComponent(steeringGraph, 0, 0, .25, 1.0);
		this.addComponent(rollGraph, .25, 0, .25, 1.0);
		this.addComponent(pitchGraph, .50, 0, .25, 1.0);
		this.addComponent(yawGraph, .75, 0, .25, 1.0);
		
		// Subscriber for drive control updates
		new Subscriber(NodeChannel.STEERING.getMsgPath(), new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				steeringGraph.updateGraph(((SteeringMeasurement)m).getAngle());
			    Gui.getInstance().fixPaint();
			}
		});
		
		// Subscriber for Imu updates
		new Subscriber(NodeChannel.IMU.getMsgPath(), new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
			    Gui.getInstance().fixPaint();
				// TODO handle imu updates for graphs
			}
		});
	}
}
