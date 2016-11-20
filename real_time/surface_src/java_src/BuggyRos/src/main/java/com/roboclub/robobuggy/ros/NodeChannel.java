package com.roboclub.robobuggy.ros;

import java.util.ArrayList;

public enum NodeChannel {

	GPS("gps", true), //The most recent gps coordinate

	LOGIC_NOTIFICATION("logic_notification", true),

	PUSHBAR_CAMERA("push_bar_camera", true),
	VISION("vision", true),

	AUTO("auto", true),
	GUI_LOGGING_BUTTON("logging_button", true),
	RESET("reset", true),
	STATE("state", true),
	SIMULATION("simulation", true),

	POSE("pose", true),
	SIM_POSE("sim_pose",true), //represents the hidden simulated ground truth
	NODE_STATUS("node_status", false),
	PATH_PLANNER("path_planner", true),

	RC("rc_angle", true), //?
	FP_HASH("fp_hash", true),
	STEERING("steering", true), //The current angle that low level is steering to
	DRIVE_CTRL("drive_ctrl", true),
	BRAKE_CTRL("commanded brake", true),
	ENCODER("encoder", true),
	STEERING_COMMANDED("commanded_steering", true), // the angle that we are commanding the front wheel turn to
	BATTERY("battery", true),
	MEGATIME("mega_time", true),
	DEVICE_ID("device_id", true),
	BRAKE_STATE("brake_state", true),
	AUTON_STATE("auton_state", true),
	ENCODER_RESET("encoder_reset", true),
	AUTON_BRAKE_STATE("auton_brake_state", true),
	TELEOP_BRAKE_STATE("teleop_brake_state", true),

	IMU("imu", true), //The most recent imu measurement
	HILL_CREST_IMU("hill_crest_imu",true),
	IMU_MAGNETIC("mag", true),
	IMU_LINEAR_ACC("imu_linear_acc", true),
	IMU_ANG_VEL("imu_ang_vel", true),
	IMU_COMPASS("imu_compass", true),
	IMU_ANG_POS("imu_ang_pos", true),
	IMU_ACCELERATION("imu_acceleration", true),
	IMU_INCLINATION("imu_inclination", true),

	UNKNOWN_CHANNEL("unknown", false),
	;





	private String rstPath;
	private String msgPath;
	private String statePath;
	private String name;
	private boolean shouldLog;

	private NodeChannel(String name, boolean shouldLog) {
		this.rstPath = "sensors/" + name + "/reset";
		this.msgPath = "sensors/" + name;
		this.statePath = "sensor/" + name + "/state";
		this.name = name;
		this.shouldLog = shouldLog;
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
		ArrayList<NodeChannel> logChannels = new ArrayList<>();
		for (NodeChannel channel : NodeChannel.values()) {
			if (channel.shouldLog) {
				logChannels.add(channel);
			}
		}
		return logChannels.toArray(new NodeChannel[logChannels.size()]);
	}

}
