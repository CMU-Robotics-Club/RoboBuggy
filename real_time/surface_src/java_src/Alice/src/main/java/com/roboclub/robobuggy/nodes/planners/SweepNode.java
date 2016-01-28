package com.roboclub.robobuggy.nodes.planners;

import com.roboclub.robobuggy.messages.DriveControlMessage;
import com.roboclub.robobuggy.messages.PoseMessage;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;

import java.util.Date;

/**
 * Created by vivaanbahl on 1/27/16.
 */
public class SweepNode extends PathPlannerNode {

    private double currentCommandedSteeringAngle = 0;
    private static final double STEERING_ANGLE_LOWER_BOUND = -10;
    private static final double STEERING_ANGLE_UPPER_BOUND = 10;
    private static final double STEERING_ANGLE_INCREMENT = 0.1;
    private Publisher steeringPublisher;

    /**
     * Construct a new {@link PathPlannerNode}
     *
     * @param channel {@link NodeChannel} on which to broadcast status
     *                information about the node
     */
    public SweepNode(NodeChannel channel) {
        super(channel);

        steeringPublisher = new Publisher(NodeChannel.DRIVE_CTRL.getMsgPath());

        while (currentCommandedSteeringAngle > STEERING_ANGLE_LOWER_BOUND) {
            currentCommandedSteeringAngle -= STEERING_ANGLE_INCREMENT;
            steeringPublisher.publish(new DriveControlMessage(new Date(), currentCommandedSteeringAngle));

            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }
        while (currentCommandedSteeringAngle < STEERING_ANGLE_UPPER_BOUND) {
            currentCommandedSteeringAngle += STEERING_ANGLE_INCREMENT;
            steeringPublisher.publish(new DriveControlMessage(new Date(), currentCommandedSteeringAngle));

            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }
    }

    @Override
    protected void updatePositionEstimate(PoseMessage m) {
        // do nothing here, this is just a simple sweep
        // we don't need to know the position
    }

    @Override
    protected double getCommandedSteeringAngle() {
        return currentCommandedSteeringAngle;
    }

    @Override
    protected boolean getDeployBrakeValue() {
        return false;
    }

    @Override
    public String getName() {
        return "Sweep Test";
    }
}
