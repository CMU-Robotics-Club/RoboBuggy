package com.roboclub.robobuggy.main;

import java.awt.Color;
import java.awt.GridLayout;

import javax.swing.BorderFactory;
import javax.swing.JLabel;
import javax.swing.SwingConstants;

import com.roboclub.robobuggy.serial.SerialEvent;
import com.roboclub.robobuggy.serial.SerialListener;

public class GpsPanel extends SerialPanel {
	private static final long serialVersionUID = 1399590586061060311L;
	private static final char[] HEADER = {'T', 'E', 'S', 'T'};
	private static final int HEADER_LEN = 4;
	
	private double longitude;
	private double latitude;
	
	public GpsPanel() {
		super("GPS", 9600, HEADER, HEADER_LEN);
		super.addListener(new GpsListener());
		
		//this.setBorder(BorderFactory.createLineBorder(Color.black));
		//this.setLayout(new GridLayout(4, 1));
	}
	
	private class GpsListener implements SerialListener {
		@Override
		public void onEvent(SerialEvent event) {
			longitude = 0.0;
			latitude = 0.0;
		}
	}
}