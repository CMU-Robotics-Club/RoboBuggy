package com.roboclub.robobuggy.ui;

import javax.swing.JPanel;

import org.opencv.highgui.VideoCapture;

public class CameraPanel extends JPanel {
	private static final long serialVersionUID = 2045798342979823126L;

	private static VideoCapture camera;	
	
	public CameraPanel( int camera_id ) {
		//JLabel cam_message = new JLabel("Cam STUFF goes here",SwingConstants.CENTER);
		//camPanel.add(cam_message);	
	}
}