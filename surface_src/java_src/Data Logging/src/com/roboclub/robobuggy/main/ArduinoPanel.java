package com.roboclub.robobuggy.main;

import com.roboclub.robobuggy.serial.SerialEvent;
import com.roboclub.robobuggy.serial.SerialListener;

public class ArduinoPanel extends SerialPanel {
	private static final long serialVersionUID = -929040896215455343L;
	private static final char[] HEADER = {'a'};
	private static final int HEADER_LEN = 1;
	private static final int BAUDRATE = 9600;

	public ArduinoPanel() throws Exception {
		super("ARDUINO", BAUDRATE, HEADER, HEADER_LEN);
		super.addListener(new ArduinoListener());
	}
	
	private class ArduinoListener implements SerialListener {
		@Override
		public void onEvent(SerialEvent event) {
			// TODO Parse arduino data
		}
	}
}