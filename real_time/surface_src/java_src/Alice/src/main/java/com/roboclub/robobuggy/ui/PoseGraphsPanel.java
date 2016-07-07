package com.roboclub.robobuggy.ui;

import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.NodeChannel;

/**
 * graphs for the pose
 */
public class PoseGraphsPanel extends RobobuggyGUIContainer {

    /**
     * makes the pose graphs panels
     */
    public PoseGraphsPanel() {
        this.addComponent(new PoseViewer(NodeChannel.POSE, true), 0.0, 0.0, .5, 1.0);
        this.addComponent(new RoboBuggyGraph("Encoder", NodeChannel.ENCODER.getMsgPath(), new RoboBuggyGraph.GetGraphValues() {

            @Override
            public double getY(Message m) {
                EncoderMeasurement encM = (EncoderMeasurement) m;
                return encM.getDistance();
            }

            @Override
            public double getX(Message m) {
                EncoderMeasurement encM = (EncoderMeasurement) m;
                return encM.getTimestamp().getTime();
            }
        }), 0.5, 0.0, 0.5, 0.5);
        this.addComponent(new RoboBuggyGraph("Steering", NodeChannel.STEERING.getMsgPath(), new RoboBuggyGraph.GetGraphValues() {

            @Override
            public double getY(Message m) {
                SteeringMeasurement strM = (SteeringMeasurement) m;
                return strM.getAngle();
            }

            @Override
            public double getX(Message m) {
                SteeringMeasurement strM = (SteeringMeasurement) m;
                return strM.getTimestamp().getTime();
            }
        }), 0.5, 0.0, 0.5, 0.5);


    }
}
