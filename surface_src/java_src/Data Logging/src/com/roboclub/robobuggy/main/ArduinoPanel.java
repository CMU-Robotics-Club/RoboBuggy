package com.roboclub.robobuggy.main;

import com.roboclub.robobuggy.serial.SerialEvent;
import com.roboclub.robobuggy.serial.SerialListener;

public class ArduinoPanel extends SerialPanel {
	private static final long serialVersionUID = -929040896215455343L;
	private static final char[] HEADER = {'F', 'C'};
	private static final int HEADER_LEN = 2;
	private static final int BAUDRATE = 9600;
	
	private static final int ENC_BYTE_ONE_TICK_LAST = 0;
	private static final int ENC_BYTE_TW0_TICK_LAST = 1;
	private static final int ENC_BYTE_ONE_TICK_RESET = 2;
	private static final int ENC_BYTE_TWO_TICK_RESET = 3;
	private static final int ENC_TIMESTAMP_ONE = 4;
	private static final int ENC_TIMESTAMP_TWO = 5;
	
	private int encResetTmp;
	private long encReset;
	private int encTickLastTmp;
	private long encTickLast;
	private int encTimeTmp;
	private long encTime;

	public ArduinoPanel() throws Exception {
		super("ARDUINO", BAUDRATE, HEADER, HEADER_LEN);
		super.addListener(new ArduinoListener());
	}
	
	private class ArduinoListener implements SerialListener {
		@Override
		public void onEvent(SerialEvent event) {
			char[] tmp = event.getBuffer();
			
			if (tmp != null && event.getLength() > HEADER_LEN) {
				String curVal = "";
				for (int i = (HEADER_LEN+1); i < event.getLength(); i++ ) {
					if (tmp[i] == ',' || tmp[i] == '\n') {
						try {
							switch (Integer.parseInt(curVal.substring(0,2), 16)) {
							case ENC_BYTE_ONE_TICK_LAST:
								encTickLastTmp = Integer.parseInt(curVal.substring(2,6), 16);
								break;
							case ENC_BYTE_TW0_TICK_LAST:
								encTickLast = (long)(encTickLastTmp << 0x32) & Long.parseLong(curVal.substring(2,6), 16);
								System.out.println("Tick Last: " + encTickLast);
								break;
							case ENC_BYTE_ONE_TICK_RESET:
								encResetTmp = Integer.parseInt(curVal.substring(2,6), 16);
								break;
							case ENC_BYTE_TWO_TICK_RESET:
								encReset = (long)(encResetTmp << 0x32) & Long.parseLong(curVal.substring(2,6), 16);
								System.out.println("Reset: " + encReset);
								break;
							case ENC_TIMESTAMP_ONE:
								encTimeTmp = Integer.parseInt(curVal.substring(2,6), 16);
								break;
							case ENC_TIMESTAMP_TWO:
								encTime = (long)(encTimeTmp << 0x32) & Long.parseLong(curVal.substring(2,6), 16);
								System.out.println("Time: " + encTime);
								break;
							default:
								return;
							}
						
							curVal = "";
						} catch (Exception e) {
							System.out.println("Failed to parse gps message");
							return;
						}
						
					} else {
						curVal += tmp[i];
					}
				}

				//TODO redraw now
			}
		}
	}
}