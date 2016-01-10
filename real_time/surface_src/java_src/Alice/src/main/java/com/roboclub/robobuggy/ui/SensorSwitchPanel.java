package com.roboclub.robobuggy.ui;

import java.awt.Color;
import java.awt.Font;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.BorderFactory;
import javax.swing.JButton;
import javax.swing.JPanel;

import com.roboclub.robobuggy.ros.SensorChannel;

public class SensorSwitchPanel extends RoboBuggyGUIContainer{
	
	SensorSwitch gps_switch;
	SensorSwitch vision_switch;
	SensorSwitch encoders_switch;
	SensorSwitch imu_switch;
	SensorSwitch controls_switch;
	SensorSwitch autonomous_switch;
	JButton display;
	
	public SensorSwitchPanel() {
	gps_switch = new SensorSwitch("GPS", SensorChannel.GPS);
	vision_switch = new SensorSwitch("VISION", SensorChannel.VISION);
	encoders_switch = new SensorSwitch("ENCODERS", SensorChannel.ENCODER);
	imu_switch = new SensorSwitch("IMU", SensorChannel.IMU);
	controls_switch = new SensorSwitch("CONTROLS", SensorChannel.DRIVE_CTRL);
	autonomous_switch = new SensorSwitch("AUTO", SensorChannel.AUTO);
	
	this.addComponet(autonomous_switch, 0, 0, 1, .16);
	this.addComponet(gps_switch, 0, .16, 1, .16);
	this.addComponet(imu_switch, 0, .32, 1, .16);
	this.addComponet(encoders_switch, 0, .48, 1, .16);
	this.addComponet(controls_switch, 0, .64, 1, .16);
	this.addComponet(vision_switch, 0, .80, 1, .16);
	}
}
