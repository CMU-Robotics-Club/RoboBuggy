package com.roboclub.robobuggy.ros;

public enum NodeChannel {
	GPS("gps"),    								//The most recent gps coordinate 
	IMU("imu"),									//The most recent imu measurement 
	IMU_MAGNETIC("mag"),
	RC("rc_angle"),								//?
	STEERING("steering"),						//The current angle that low level is steering to 
	PUSHBAR_CAMERA("push_bar_camera"),
	BRAKE("brake"),
	LOGIC_NOTIFICATION("logic_notification"),
	DRIVE_CTRL("drive_ctrl"),
	BRAKE_CTRL("commanded brake"),
	VISION("vision"),
	ENCODER("encoder"),
	AUTO("auto"),
	GUI_LOGGING_BUTTON("logging_button"),
	STEERING_COMMANDED("commanded_steering"),   // the angle that we are commanding the front wheel turn to 
	FP_HASH("fp_hash"),
	POSE("pose"),
	RESET("reset"),
	STATE("state"),
	SIMULATION("simulation"),
	NODE_STATUS("node_status"),
	PATH_PLANNER("path_planner"),
	BATTERY("battery"),
	ENCODERTIME("encoder_time"),
	DEVICE_ID("device_id"),
	BRAKE_STATE("brake_state"),
	AUTON_STATE("auton_state"),
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

	public static NodeChannel[] getLoggingChannels() {
		return new NodeChannel[] {  GPS,
									IMU,
									IMU_MAGNETIC,
									PUSHBAR_CAMERA,
									STEERING,
									BRAKE,
									LOGIC_NOTIFICATION,
									DRIVE_CTRL,
									BRAKE_CTRL,
									VISION,
									ENCODER,
									AUTO,
									GUI_LOGGING_BUTTON,
									STEERING_COMMANDED,
									FP_HASH,
									POSE,
									RESET,
									STATE
								};
	}

}
