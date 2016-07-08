package com.roboclub.robobuggy.ui;

import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.NodeChannel;

/**
 * Imu panel - shows values from the IMU
 */
public class ImuPanel extends RobobuggyGUIContainer {

    /**
     * makes a new imupanel
     */
    public ImuPanel() {
        this.addComponent(new RoboBuggyGraph("Yaw", NodeChannel.IMU.getMsgPath(), new RoboBuggyGraph.GetGraphValues() {

            @Override
            public double getY(Message m) {
                ImuMeasurement imuM = (ImuMeasurement) m;
                return imuM.getYaw();
            }

            @Override
            public double getX(Message m) {
                ImuMeasurement imuM = (ImuMeasurement) m;
                return imuM.getTimestamp().getTime();
            }
        }), 0.0, 0.0, .5, .5);

        this.addComponent(new RoboBuggyGraph("Pitch", NodeChannel.IMU.getMsgPath(), new RoboBuggyGraph.GetGraphValues() {

            @Override
            public double getY(Message m) {
                ImuMeasurement imuM = (ImuMeasurement) m;
                return imuM.getPitch();
            }

            @Override
            public double getX(Message m) {
                ImuMeasurement imuM = (ImuMeasurement) m;
                return imuM.getTimestamp().getTime();
            }
        }), 0.5, 0.0, .5, .5);

        this.addComponent(new RoboBuggyGraph("Roll", NodeChannel.IMU.getMsgPath(), new RoboBuggyGraph.GetGraphValues() {

            @Override
            public double getY(Message m) {
                ImuMeasurement imuM = (ImuMeasurement) m;
                return imuM.getRoll();
            }

            @Override
            public double getX(Message m) {
                ImuMeasurement imuM = (ImuMeasurement) m;
                return imuM.getTimestamp().getTime();
            }
        }), 0.0, 0.5, .5, .5);

    }

}
