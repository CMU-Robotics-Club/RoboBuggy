package com.roboclub.robobuggy.serial2;

public class RBSerial {
	/*ENC_TICKS_LAST(0),
	ENC_TICKS_RESET(1),
	ENC_TIMESTAMP(2);*/
	// TODO use enums instead, vs. below
	
	public static final int MSG_LEN = 6;
	
	private static final char ENC_RESET = (char)0;
	private static final char ENC_TICK = (char)1;
	private static final char ENC_TIME = (char)2;
	
	private static final char STEERING = (char)20;
	private static final char BRAKE = (char)21;
	private static final char AUTO = (char)22;
	private static final char BATTERY = (char)23;
	
	private static final char ERROR = (char)254;
	private static final char MSG_ID = (char)255;
	
	private RBSerial(String name) {
		this.msgPath = name;
	}

	
	static Pair<int, RBSerialMessage> peel(byte[]) {
		return new Pair
	}
	
	/* Methods for reading from Serial */
	@Override
	public boolean validId(char value) {
		switch (value) {
			case ENC_TIME:
			case ENC_RESET:
			case ENC_TICK:
			case ERROR:
			case MSG_ID:
				return true;
			default:
				return false;
		}
	}

	protected int parseInt(char x0, char x1, char x2, char x3) {
		int val = 0;
		val |= x0;
		val = val << 0x8;
		val |= x1;
		val = val << 0x8;
		val |= x2;
		val = val << 0x8;
		val |= x3;
		
		return val;
	}
	
	
	private void something() {
		int value = parseInt(inputBuffer[1], inputBuffer[2],
				inputBuffer[3], inputBuffer[4]);
		try {
			switch (inputBuffer[0]) {
			case ENC_TIME:
				encTime = value;
				break;
			case ENC_RESET:
				encReset = value;
				break;
			case ENC_TICK:
				encTicks = value;
				estimateVelocity();
				break;
			case ERROR:
				// TODO handle errors
				break;
			}
		} catch (Exception e) {
			System.out.println("Encoder Exception on port: " + this.getName());
			if (this.currState != SensorState.FAULT) {
				this.currState = SensorState.FAULT;
				statePub.publish(new StateMessage(this.currState));
			}
			return;
		}
	}
	}
}