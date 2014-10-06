package com.roboclub.robobuggy.main;

import java.util.Date;

import com.roboclub.robobuggy.logging.RobotLogger;
import com.roboclub.robobuggy.serial.SerialEvent;
import com.roboclub.robobuggy.serial.SerialListener;

public class ArduinoPanel extends SerialPanel {
	private static final long serialVersionUID = -929040896215455343L;
	private static final char[] HEADER = {(char)0xFC};
	private static final int HEADER_LEN = 1;
	private static final int MSG_LEN = 8;
	private static final int BAUDRATE = 9600;
	
	private static final char ENC_BYTE_ONE_TICK_LAST = 0;
	private static final char ENC_BYTE_TW0_TICK_LAST = 1;
	private static final char ENC_BYTE_ONE_TICK_RESET = 2;
	private static final char ENC_BYTE_TWO_TICK_RESET = 3;
	private static final char ENC_TIMESTAMP_ONE = 4;
	private static final char ENC_TIMESTAMP_TWO = 5;
	
	private int encResetTmp;
	private long encReset;
	private int encTickLastTmp;
	private long encTickLast;
	private int encTimeTmp;
	private long encTime;

	public ArduinoPanel() {
		super("ARDUINO", BAUDRATE, HEADER, HEADER_LEN);
	
		
		
		if (!this.isConnected()) return;
		
		super.addListener(new ArduinoListener());
	}
	
	private void logData() {
		if (Gui.GetPlayPauseState()) {
			RobotLogger rl = RobotLogger.getInstance();
		    Date now = new Date();
		    long time_in_millis = now.getTime();
		    rl.sensor.logEncoder(time_in_millis, encTickLast, encReset, encTime);
		}
	}
	
	private int parseData(char[] data, int ind) {
		int tmp = data[0] << (HEADER_LEN + ind + 2);
		tmp |= data[1] << (HEADER_LEN + ind + 3);
		tmp |= data[1] << (HEADER_LEN + ind + 4);
		tmp |= data[1] << (HEADER_LEN + ind + 5);
		
		return tmp;
	}
	
	public void writeAngle(int angle) {
		if (angle >= 0 && angle <= 180) {
			if(isConnected()) {
				byte[] msg = {'T', 'H', 'a','\n'};
				msg[2] = (byte)angle;
				this.port.serialWrite(msg);
			}
		}
	}
	
	private class ArduinoListener implements SerialListener {
		@Override
		public void onEvent(SerialEvent event) {
			if (!Gui.InPlayBack() 
					&& (Gui.GetGraphState() || Gui.GetPlayPauseState())) {
				char[] tmp = event.getBuffer();
				int tmp_len = event.getLength();
				
				for (int i = 0; i < event.getLength(); i++ ) {
					if (tmp[i] == HEADER[0] && (tmp_len - i) >= MSG_LEN) {
						switch(tmp[i+1]) {
						case ENC_BYTE_ONE_TICK_LAST:
							encTickLastTmp = parseData(tmp, i);
							i += MSG_LEN;
							break;
						case ENC_BYTE_TW0_TICK_LAST:
							encTickLast = (((long)0 | encTickLastTmp) << 0x32) & parseData(tmp, i);
							i += MSG_LEN;
							System.out.println("Tick Last: " + encTickLast);
							logData();
							break;
						case ENC_BYTE_ONE_TICK_RESET:
							encResetTmp = parseData(tmp, i);
							i += MSG_LEN;
							break;
						case ENC_BYTE_TWO_TICK_RESET:
							encReset = (((long)0 | encResetTmp << 0x32)) & parseData(tmp, i);
							i += MSG_LEN;
							System.out.println("Reset: " + encReset);
							logData();
							break;
						case ENC_TIMESTAMP_ONE:
							encTimeTmp = parseData(tmp, i);
							i += MSG_LEN;
							break;
						case ENC_TIMESTAMP_TWO:
							encTime = (((long)0 | encTimeTmp) << 0x32) & parseData(tmp, i);
							i += MSG_LEN;
							System.out.println("Time: " + encTime);
							logData();
							break;
						default:
							System.out.println("How did you get here");
							return;
						}
					}	
				}
			}
		}
	}
}