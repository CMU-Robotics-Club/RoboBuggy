package com.roboclub.robobuggy.ros;

public enum SensorChannel {
	GPS("gps"),
	IMU("imu"),
	RC("rc_angle"),
	STEERING("steering"),
	BRAKE("brake"),
	LOGIC_EXCEPTION("logic_exception"),
	DRIVE_CTRL("drive_ctrl"),
	BRAKE_CTRL("commanded brake"),
	VISION("vision"),
	ENCODER("encoder"),
	AUTO("auto"),
	GUI_LOGGING_BUTTON("logging_button"),
	STEERING_COMMANDED("commanded_steering"),
	FP_HASH("fp_hash");
	;
	
	private String rstPath;
	private String msgPath;
	private String statePath;
	
	private SensorChannel(String name) {
		this.rstPath = "sensors/" + name + "/reset";
		this.msgPath = "sensors/" + name;
		this.statePath = "sensor/" + name + "/state";
	}
	
	public String getRstPath() {
		return this.rstPath;
	}
	
	public String getMsgPath() {
		return this.msgPath;
	}
	
	public String getStatePath() {
		return this.statePath;
	}
}
