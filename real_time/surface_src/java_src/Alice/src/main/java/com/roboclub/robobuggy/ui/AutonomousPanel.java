package com.roboclub.robobuggy.ui;

import com.roboclub.robobuggy.messages.DriveControlMessage;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ui.RoboBuggyGraph.getGraphValues;

/**
 * 
 * @author Trevor Decker
 *
 */
public class AutonomousPanel extends RobobuggyGUIContainer{
	
	public AutonomousPanel() {
		this.addComponent(new RoboBuggyGraph("Drive Control", NodeChannel.DRIVE_CTRL.getMsgPath(), new getGraphValues() {
			
			@Override
			public double getY(Message m) {
				DriveControlMessage steerM = (DriveControlMessage)m;
				return steerM.getAngleDouble();
			}
			
			@Override
			public double getX(Message m) {
				DriveControlMessage steerM = (DriveControlMessage)m;
				return steerM.getTimestamp().getTime();

			}
		}),0,0,1,1);
		
	}

}
