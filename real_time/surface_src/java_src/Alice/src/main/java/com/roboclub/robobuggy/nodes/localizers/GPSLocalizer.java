package com.roboclub.robobuggy.nodes.localizers;

import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyBaseNode;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyDecoratorNode;
import com.roboclub.robobuggy.nodes.baseNodes.NodeState;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.Subscriber;
import com.roboclub.robobuggy.ui.LocTuple;

import java.util.Date;

/**
 * {@link BuggyDecoratorNode} used create {@link PoseMessage}s based upon GPS 
 * sensor input
 * 
 * @author Zachary Dawson
 *
 */
public final class GPSLocalizer extends BuggyDecoratorNode {

	private Publisher posePub;
	private LocTuple lastReading;
	
	/**
	 * Construct a new {@link GPSLocalizer} object
	 * @param channel channel {@link NodeChannel} on which to broadcast status
	 *  information about the node
	 */
	public GPSLocalizer(NodeChannel channel) {
		super(new BuggyBaseNode(channel), "GPSLocalizer");
		lastReading= null;
		posePub = new Publisher(NodeChannel.POSE.getMsgPath());
	}

	/**{@inheritDoc}*/
	@Override
	protected boolean startDecoratorNode() {
		//Initialize subscriber to GPS measurements
		new Subscriber(NodeChannel.GPS.getMsgPath(), new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				GpsMeasurement gpsM = (GpsMeasurement)m;
				LocTuple reading = new LocTuple(gpsM.getLatitude(), gpsM.getLongitude());
				if(lastReading != null) {
					LocTuple diff = LocTuple.subtract(lastReading, reading);
					posePub.publish(new GPSPoseMessage(new Date(), reading.getLatitude(),
							reading.getLongitude(), diff.getHeadingAngle()));
				}
				lastReading = reading;
				
				//Feed the watchdog
				setNodeState(NodeState.ON);
			}
			
		});
		return true;
	}

	/**{@inheritDoc}*/
	@Override
	protected boolean shutdownDecoratorNode() {
		//Always return true as this node always shuts down successfully
		return true;
	}

}
