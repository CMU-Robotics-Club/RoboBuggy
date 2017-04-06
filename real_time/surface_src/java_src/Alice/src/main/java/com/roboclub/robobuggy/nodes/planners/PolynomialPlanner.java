package com.roboclub.robobuggy.nodes.planners;

import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.ros.NodeChannel;

import java.util.ArrayList;

/**
 * Created by vivaanbahl on 4/4/17.
 */
public class PolynomialPlanner extends PathPlannerNode {

    private ArrayList<GpsMeasurement> waypoints;

    /**
     * Construct a new {@link PathPlannerNode}
     *
     */
    public PolynomialPlanner(ArrayList<GpsMeasurement> waypoints) {
        super(NodeChannel.PATH_PLANNER);
        this.waypoints = waypoints;
    }

    @Override
    protected void updatePositionEstimate(GPSPoseMessage m) {

    }

    @Override
    protected double getCommandedSteeringAngle() {
        return 0;
    }

    @Override
    protected boolean getDeployBrakeValue() {
        return false;
    }
}
