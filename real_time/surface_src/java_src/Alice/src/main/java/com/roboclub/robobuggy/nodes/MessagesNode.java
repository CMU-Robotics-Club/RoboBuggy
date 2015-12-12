package com.roboclub.robobuggy.nodes;

import java.util.ArrayList;

import com.roboclub.robobuggy.main.MessageLevel;
import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.messages.RobobuggyLogicExceptionMeasurment;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
//This node controls what messages are to be displayed in the console 
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.ros.Subscriber;

public class MessagesNode implements Node{
	//is a list of what message levels to show, if a messagelevel is in this list then it will be displayed
	private static ArrayList<MessageLevel> messageLevelsToShow;

	public MessagesNode() {
		messageLevelsToShow = new ArrayList<MessageLevel>();
		
		new Subscriber(SensorChannel.LOGIC_EXCEPTION.getMsgPath(), new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				updateConsole((RobobuggyLogicExceptionMeasurment)m);
			}
		});
	}
	
	public void updateConsole(RobobuggyLogicExceptionMeasurment m){
		//TODO add message to a console that is on the gui 
		for(MessageLevel messageLevel : messageLevelsToShow){
			if(messageLevel.equals(m.getMessageLevel())){
				System.out.println(m.getMessage());
			}
		}
		
	}
	
	@Override
	public boolean shutdown() {
		// TODO Auto-generated method stub
		return false;
	}


}
