package com.roboclub.robobuggy.ui;

import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.SwingConstants;

public class GpsPanel extends JPanel {
	private static final long serialVersionUID = 1399590586061060311L;

	public GpsPanel() {
		JPanel gpsPanel = new JPanel();
		JLabel gps_message = new JLabel("GPS STUFF goes here",SwingConstants.CENTER);
		gpsPanel.add(gps_message);		
	}
}