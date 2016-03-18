package com.roboclub.robobuggy.ui;

import com.roboclub.robobuggy.messages.MagneticMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.NodeChannel;

/**
 * Imu panel - shows values from the IMU
 */
public class ImuPanel extends RobobuggyGUIContainer{

	/**
	 * makes a new imupanel
	 */
	public ImuPanel(){
		this.addComponent(new RoboBuggyGraph("X", NodeChannel.IMU_MAGNETIC.getMsgPath(), new RoboBuggyGraph.GetGraphValues() {
			
			@Override
			public double getY(Message m) {
				MagneticMeasurement imuM = (MagneticMeasurement)m;
				return imuM.getX();
			}
			
			@Override
			public double getX(Message m) {
				MagneticMeasurement imuM = (MagneticMeasurement)m;
				return imuM.getTimestamp().getTime();
			}
		}), 0.0, 0.0, .5, .5);
		
		this.addComponent(new RoboBuggyGraph("Y", NodeChannel.IMU_MAGNETIC.getMsgPath(), new RoboBuggyGraph.GetGraphValues() {
			
			@Override
			public double getY(Message m) {
				MagneticMeasurement imuM = (MagneticMeasurement)m;
				return imuM.getY();
			}
			
			@Override
			public double getX(Message m) {
				MagneticMeasurement imuM = (MagneticMeasurement)m;
				return imuM.getTimestamp().getTime();
			}
		}), 0.5, 0.0, .5, .5);
		
		this.addComponent(new RoboBuggyGraph("Z", NodeChannel.IMU_MAGNETIC.getMsgPath(), new RoboBuggyGraph.GetGraphValues() {
			
			@Override
			public double getY(Message m) {
				MagneticMeasurement imuM = (MagneticMeasurement)m;
				return imuM.getz();
			}
			
			@Override
			public double getX(Message m) {
				MagneticMeasurement imuM = (MagneticMeasurement)m;
				return imuM.getTimestamp().getTime();
			}
		}), 0.0, 0.5, .5, .5);
		/*
		this.addComponent(new RoboBuggyGraph("Yaw", NodeChannel.IMU.getMsgPath(), new RoboBuggyGraph.GetGraphValues() {
			
			@Override
			public double getY(Message m) {
				ImuMeasurement imuM = (ImuMeasurement)m;
				return imuM.getYaw();
			}
			
			@Override
			public double getX(Message m) {
				ImuMeasurement imuM = (ImuMeasurement)m;
				return imuM.getTimestamp().getTime();
			}
		}), 0.0, 0.0, .5, .5);
		
		this.addComponent(new RoboBuggyGraph("Pitch", NodeChannel.IMU.getMsgPath(), new RoboBuggyGraph.GetGraphValues() {
			
			@Override
			public double getY(Message m) {
				ImuMeasurement imuM = (ImuMeasurement)m;
				return imuM.getPitch();
			}
			
			@Override
			public double getX(Message m) {
				ImuMeasurement imuM = (ImuMeasurement)m;
				return imuM.getTimestamp().getTime();
			}
		}), 0.5, 0.0, .5, .5);
		
		this.addComponent(new RoboBuggyGraph("Roll", NodeChannel.IMU.getMsgPath(), new RoboBuggyGraph.GetGraphValues() {
			
			@Override
			public double getY(Message m) {
				ImuMeasurement imuM = (ImuMeasurement)m;
				return imuM.getRoll();
			}
			
			@Override
			public double getX(Message m) {
				ImuMeasurement imuM = (ImuMeasurement)m;
				return imuM.getTimestamp().getTime();
			}
		}), 0.0, 0.5, .5, .5);
		*/
	}
	
}
