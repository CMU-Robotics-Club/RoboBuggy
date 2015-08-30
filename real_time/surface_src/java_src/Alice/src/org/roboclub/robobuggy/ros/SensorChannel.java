package org.roboclub.robobuggy.ros;

/**
 * TODO document 
 * @author Matt Sebek
 *
 */
public enum SensorChannel {
	GPS("gps"),
	IMU("imu"),
    RC("rc_angle"),
    STEERING("steering"),
    BRAKE("brake"),
	DRIVE_CTRL("drive_ctrl"),
	VISION("vision"),
	ENCODER("encoder"),
	GUI_LOGGING_BUTTON("logging_button"),
	GUI_CONSOLE_TEXT("console_text"),
	AUTO("auto");
	
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
