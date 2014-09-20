package com.roboclub.robobuggy;

public interface DataLog {

	public enum MessageType {
		ACCELEROMETER,
		GPS,
		ENCODER,
		COMMANDED_STEERING_ANGLE,
		CAMERA,
		OTHER
	}
	
	// Log a data message
	// Note that the user has to interpret this correctly...which makes me sad a bit.
	//
	public void log(MessageType type, byte[] message);
}
