package com.roboclub.robobuggy.nodes.planners;

import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.main.Util;
import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.nodes.localizers.LocalizerUtil;
import com.roboclub.robobuggy.ros.NodeChannel;

import java.util.ArrayList;
import java.util.Date;

/**
 * Plans a path based on a set of waypoints
 */
public class WayPointFollowerPlanner extends PathPlannerNode {
    private ArrayList<GpsMeasurement> wayPoints;
    private GPSPoseMessage pose; //TODO change this to a reasonable type

    /**
     * @vivaanbahl TESTING CODE ONLY
     * REMOVE FOR PROD PUSH
     */
    public static GpsMeasurement currentWaypoint = new GpsMeasurement(0, 0);
    public static double currentCommandedAngle = 0.0;
    public static double currentDesiredHeading = 0.0;

    /**
     * @param wayPoints the list of waypoints to follow
     */
    public WayPointFollowerPlanner(ArrayList<GpsMeasurement> wayPoints) {
        super(NodeChannel.PATH_PLANNER);
        this.wayPoints = wayPoints;
        pose = new GPSPoseMessage(new Date(0), 0, 0, 0);// temp measurment
    }

    @Override
    public void updatePositionEstimate(GPSPoseMessage m) {
        pose = m;

    }

    //find the closest way point
    //TODO turn into a binary kdtree search
    private static int getClosestIndex(ArrayList<GpsMeasurement> wayPoints, GPSPoseMessage currentLocation) {
        double min = Double.MAX_VALUE; //note that the brakes will definitely deploy at this

        int closestIndex = -1;
        for (int i = 0; i < wayPoints.size(); i++) {
            GPSPoseMessage gpsPoseMessage = wayPoints.get(i).toGpsPoseMessage(0);
            double d = GPSPoseMessage.getDistance(currentLocation, gpsPoseMessage);
            if (d < min) {
                min = d;
                closestIndex = i;
            }
        }
        return closestIndex;
    }

    @Override
    public double getCommandedSteeringAngle() {
        // determines the angle at which to move every 50 milliseconds
        int closestIndex = getClosestIndex(wayPoints, pose);
        if (closestIndex == -1) {
            new RobobuggyLogicNotification("HELP no closest index", RobobuggyMessageLevel.EXCEPTION);
            return 17433504; //A dummy value that we can never get
        }

        double lookahead = 15; //meters
        //pick the first point that is at least lookahead away, then point buggy toward it
        int targetIndex = closestIndex;
        double distanceFromMessage = 0;
        do {
            targetIndex++;
            distanceFromMessage = GPSPoseMessage.getDistance(pose, wayPoints.get(targetIndex).toGpsPoseMessage(0));
        } while (targetIndex < wayPoints.size() && distanceFromMessage < lookahead);

        //if we are out of points then just go straight
        if (targetIndex >= wayPoints.size()) {
            new RobobuggyLogicNotification("HELP out of points", RobobuggyMessageLevel.EXCEPTION);
            return 0;
        }

        GpsMeasurement targetPoint = wayPoints.get(targetIndex);
        currentWaypoint = targetPoint;

        //find a path from our current location to that point
        double deltaLong = targetPoint.getLongitude() - pose.getLongitude();
        double deltaLat = targetPoint.getLatitude() - pose.getLatitude();
        double deltaLatMeters = LocalizerUtil.convertLatToMeters(deltaLat);
        double deltaLongMeters = LocalizerUtil.convertLonToMeters(deltaLong);
        double desiredHeading = Math.atan2(deltaLatMeters, deltaLongMeters);
        currentDesiredHeading = desiredHeading;

        //find the angle we need to reach that point
        double poseHeading = pose.getHeading();
        double deltaHeading = desiredHeading - poseHeading;

        //Pure Pursuit steering controller
//        double param1 = 2 * RobobuggyKFLocalizer.WHEELBASE_IN_METERS * Math.sin(deltaHeading);
//        double param2 = 0.8 * pose.getCurrentState().get(2, 0);
//        deltaHeading = Math.atan2(param1, param2);

        //PD control of DC steering motor handled by low level
        double commandedAngle = Util.normalizeAngleRad(deltaHeading);
        currentCommandedAngle = commandedAngle;
        return commandedAngle;
    }

    @Override
    protected boolean getDeployBrakeValue() {
        // need to brake when 15 m off course
        return false;
    }


}
