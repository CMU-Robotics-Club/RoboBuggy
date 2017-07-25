package com.roboclub.robobuggy.ui;

import com.roboclub.robobuggy.ros.NodeChannel;

/**
 * Switches to toggle individual sensors
 */
public class SensorSwitchPanel extends RobobuggyGUIContainer {

    /**
     * Instantiates the switch panel
     */
    public SensorSwitchPanel() {
        SensorSwitch gpsSwitch = new SensorSwitch("GPS", NodeChannel.GPS);
        SensorSwitch visionSwitch = new SensorSwitch("VISION", NodeChannel.VISION);
        SensorSwitch encodersSwitch = new SensorSwitch("ENCODERS", NodeChannel.ENCODER);
        SensorSwitch imuSwitch = new SensorSwitch("IMU", NodeChannel.IMU);
        SensorSwitch controlsSwitch = new SensorSwitch("CONTROLS", NodeChannel.DRIVE_CTRL);
        SensorSwitch autonomousSwitch = new SensorSwitch("AUTO", NodeChannel.AUTO);

        this.addComponent(autonomousSwitch, 0, 0, 1, .16);
        this.addComponent(gpsSwitch, 0, .16, 1, .16);
        this.addComponent(imuSwitch, 0, .32, 1, .16);
        this.addComponent(encodersSwitch, 0, .48, 1, .16);
        this.addComponent(controlsSwitch, 0, .64, 1, .16);
        this.addComponent(visionSwitch, 0, .80, 1, .16);
    }
}
