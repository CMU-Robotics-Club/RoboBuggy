package com.roboclub.robobuggy.nodes.planners;

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
    //TODO turn into a binary search
    private static int getClosestIndex(ArrayList<GpsMeasurement> wayPoints, GPSPoseMessage currentLocation) {
        double min = Double.MAX_VALUE; //note that the brakes will definitely deploy at this

        int closestIndex = -1;
        for (int i = 0; i < wayPoints.size(); i++) {
            double d = GPSPoseMessage.getDistance(currentLocation, wayPoints.get(i).toGpsPoseMessage(0));
            if (d < min) {
                min = d;
                closestIndex = i;
            }
        }
        return closestIndex;
    }

    @Override
    public double getCommandedSteeringAngle() {
        int closestIndex = getClosestIndex(wayPoints, pose);
        if (closestIndex == -1) {
            return 17433504; //A dummy value that we can never get
        }


        double delta = 10; //meters
        //pick the first point that is at least delta away
        //pick the point to follow
        int targetIndex = closestIndex;
        while (targetIndex < wayPoints.size() && GPSPoseMessage.getDistance(pose, wayPoints.get(targetIndex).toGpsPoseMessage(0)) < delta) {
            targetIndex = targetIndex + 1;
        }

        //if we are out of points then just go straight
        if (targetIndex >= wayPoints.size()) {
            return 0;
        }


        GpsMeasurement targetPoint = wayPoints.get(targetIndex);

        //find a path from our current location to that point
        double dLon = targetPoint.getLongitude() - pose.getLongitude();
        double dLat = targetPoint.getLatitude() - pose.getLatitude();
        double desiredHeading = Math.toDegrees(Math.atan2(LocalizerUtil.convertLatToMeters(dLat), LocalizerUtil.convertLonToMeters(dLon)));

        // basically we want all of our angles to be in the same range, so that we don't
        // have weird wraparound
        desiredHeading = Util.normalizeAngleDeg(desiredHeading);
        double poseHeading = Util.normalizeAngleDeg(pose.getHeading());

        //find the angle we need to reach that point
        return Util.normalizeAngleDeg(desiredHeading - poseHeading);

    }

    @Override
    protected boolean getDeployBrakeValue() {
        return false;
        /*
		  int closestIndex = getClosestIndex(wayPoints,pose);

		if(closestIndex == -1){
			return true;
		}

		// if closest point is too far away throw breaks

		boolean shouldBrake =  GPSPoseMessage.getDistance(pose, wayPoints.get(closestIndex).toGpsPoseMessage(0)) >= 5.0;
		return shouldBrake;
		*/
    }


}
