package main;

import serial.SerialEvent;
import serial.SerialListener;

public class GpsPanel extends SerialPanel {
	private static final long serialVersionUID = 1399590586061060311L;
	private static final char[] HEADER = {'T', 'E', 'S', 'T'};
	private static final int HEADER_LEN = 4;
	
	private static GpsListener listener;
	private double longitude;
	private double latitude;
	
	public GpsPanel() throws Exception {
		super("GPS", 9600, HEADER, HEADER_LEN);
		
		//listener =;
		super.addListener(new GpsListener());
		
		//TODO draw gps data
	}
	
	private class GpsListener implements SerialListener {
		@Override
		public void onEvent(SerialEvent event) {
			longitude = 0.0;
			latitude = 0.0;
		}
	}
}