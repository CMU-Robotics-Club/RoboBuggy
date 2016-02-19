package com.roboclub.robobuggy.ui;

import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Subscriber;

/**
 * Imu panel - shows values from the IMU
 */
public class ImuPanel extends RobobuggyGUIContainer{

	/**
	 * makes a new imupanel
	 */
	public ImuPanel(){
		this.addComponent(new ImuYawGraph(), 0.0, 0.0, .5, .5);
		this.addComponent(new ImuRollGraph(), 0.5, 0.0, .5, .5);
		this.addComponent(new ImuPitchGraph(), 0.0, .5, .5, .5);
	}
	
}
