package com.roboclub.robobuggy.ros;

public enum NodeChannel {
	GPS("gps"),
	IMU("imu"),
	PUSHBAR_CAMERA("push_bar_camera"),
	RC("rc_angle"),
	STEERING("steering"),
	BRAKE("brake"),
	LOGIC_NOTIFICATION("logic_notification"),
	DRIVE_CTRL("drive_ctrl"),
	BRAKE_CTRL("commanded brake"),
	VISION("vision"),
	ENCODER("encoder"),
	AUTO("auto"),
	GUI_LOGGING_BUTTON("logging_button"),
	STEERING_COMMANDED("commanded_steering"),
	FP_HASH("fp_hash"),
	POSE("pose"),
	RESET("reset"),
	STATE("state"),
	UNKNOWN_CHANNEL("unknown"),
	;
	
	private String rstPath;
	private String msgPath;
	private String statePath;
	private String name;
	
	private NodeChannel(String name) {
		this.rstPath = "sensors/" + name + "/reset";
		this.msgPath = "sensors/" + name;
		this.statePath = "sensor/" + name + "/state";
		this.name = name;
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

	public String getName() {
		return this.name;
	}

	public static NodeChannel getNodeForName(String nodeName) {
		for (NodeChannel node : NodeChannel.values()) {
			if (node.getName().equals(nodeName)) {
				return node;
			}
		}
		return UNKNOWN_CHANNEL;
	}

}
