package com.roboclub.robobuggy.nodes.sensors;

import java.io.BufferedOutputStream;
import com.orsoncharts.util.json.JSONObject;
import com.roboclub.robobuggy.messages.GuiLoggingButtonMessage;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyBaseNode;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyDecoratorNode;
import com.roboclub.robobuggy.nodes.baseNodes.SerialNode;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Subscriber;
import com.roboclub.robobuggy.ui.Gui;

/**
 * {@link SerialNode} for reading in logging commands from the GUI
 * When logging begins, a new folder is created, and then logging begins
 *  to that folder
 */
public class LoggingNode extends BuggyDecoratorNode {

	private Publisher loggingButtonPub;
	// Get the folder that we're going to use

	// TODO get folder name from file.
	/**
	 * Create a new {@link LoggingNode} decorator
	 * @param channel the {@link NodeChannel} of the {@link LoggingNode}
	 */
	public LoggingNode(NodeChannel channel) {
		super(new BuggyBaseNode(channel));
	}
	
	/**{@inheritDoc}*/
	@Override
	protected boolean startDecoratorNode() {
		loggingButtonPub = new Publisher(NodeChannel.GUI_LOGGING_BUTTON.getMsgPath());
		
		new Subscriber(Gui.GuiPubSubTopics.GUI_LOG_BUTTON_UPDATED.toString(), new MessageListener() {
			@Override 
			public void actionPerformed(String topicName, Message m) {
				loggingButtonPub.publish(m);
			}
		});
		return true;
	}

	/**{@inheritDoc}*/
	@Override
	protected boolean shutdownDecoratorNode() {
		return true;
	}

	/**
	 * Called to translate a peeled message to a JSON object
	 * @param message {@link String} of the peeled message
	 * @return {@link JSONObject} representing the string
	 */
	@SuppressWarnings("unchecked")
	public static JSONObject translatePeelMessageToJObject(String message) {
		// TODO Auto-generated method stub
		JSONObject data = new JSONObject();
		JSONObject params = new JSONObject();
		if (message.contains(GuiLoggingButtonMessage.LoggingMessage.START.toString())) {
			params.put("logging_status", "start");
		}
		else {
			params.put("logging_status", "stop");
		}
		data.put("timestamp", message.split(",")[1]);
		data.put("params", params);
		return data;
	}

}
