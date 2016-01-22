package com.roboclub.robobuggy.ui;

import javax.swing.JButton;

import com.roboclub.robobuggy.ros.NodeChannel;

public class SensorSwitchPanel extends RobobuggyGUIContainer{
	
	SensorSwitch gps_switch;
	SensorSwitch vision_switch;
	SensorSwitch encoders_switch;
	SensorSwitch imu_switch;
	SensorSwitch controls_switch;
	SensorSwitch autonomous_switch;
	JButton display;
	
	public SensorSwitchPanel() {
	gps_switch = new SensorSwitch("GPS", NodeChannel.GPS);
	vision_switch = new SensorSwitch("VISION", NodeChannel.VISION);
	encoders_switch = new SensorSwitch("ENCODERS", NodeChannel.ENCODER);
	imu_switch = new SensorSwitch("IMU", NodeChannel.IMU);
	controls_switch = new SensorSwitch("CONTROLS", NodeChannel.DRIVE_CTRL);
	autonomous_switch = new SensorSwitch("AUTO", NodeChannel.AUTO);
	
	this.addComponent(autonomous_switch, 0, 0, 1, .16);
	this.addComponent(gps_switch, 0, .16, 1, .16);
	this.addComponent(imu_switch, 0, .32, 1, .16);
	this.addComponent(encoders_switch, 0, .48, 1, .16);
	this.addComponent(controls_switch, 0, .64, 1, .16);
	this.addComponent(vision_switch, 0, .80, 1, .16);
	}
}
