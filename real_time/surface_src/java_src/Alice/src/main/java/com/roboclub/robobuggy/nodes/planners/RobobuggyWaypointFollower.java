package com.roboclub.robobuggy.nodes.planners;

import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.ros.NodeChannel;

import java.util.ArrayList;

/**
 * Created by vivaanbahl on 11/4/16.
 */
public class RobobuggyWaypointFollower extends PathPlannerNode {
    private GPSPoseMessage currentPositionEsitmate;
    private ArrayList<GpsMeasurement> waypoints;

    private double currentSteeringAngle;
    private boolean currentBrakeStatus;

    /**
     * Construct a new {@link PathPlannerNode}
     *
     * @param channel {@link NodeChannel} on which to broadcast status
     *                information about the node
     */
    public RobobuggyWaypointFollower(NodeChannel channel) {
        super(channel);
        waypoints = new ArrayList<>();
    }

    @Override
    protected void updatePositionEstimate(GPSPoseMessage m) {
        currentPositionEsitmate = m;
    }

    @Override
    protected double getCommandedSteeringAngle() {
        return currentSteeringAngle;
    }

    @Override
    protected boolean getDeployBrakeValue() {
        return currentBrakeStatus;
    }
}
