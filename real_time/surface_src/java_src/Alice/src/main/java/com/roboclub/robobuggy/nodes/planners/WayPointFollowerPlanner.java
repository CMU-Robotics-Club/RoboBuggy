package com.roboclub.robobuggy.nodes.planners;

import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.main.Util;
import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.nodes.localizers.LocalizerUtil;
import com.roboclub.robobuggy.nodes.localizers.RobobuggyKFLocalizer;
import com.roboclub.robobuggy.ros.NodeChannel;

import java.util.ArrayList;
import java.util.Date;

/**
 * Plans a path based on a set of waypoints
 */
public class WayPointFollowerPlanner extends PathPlannerNode {
    private ArrayList<GpsMeasurement> wayPoints;
    private GPSPoseMessage pose; //TODO change this to a reasonable type
    private int lastClosestIndex = 0;
    private GpsMeasurement target;

    // we only want to look at the next 10 waypoints as possible candidates
    private final static int WAYPOINT_LOOKAHEAD_MAX = 50;

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
        target = wayPoints.get(0);
        pose = new GPSPoseMessage(new Date(0), 0, 0, 0);// temp measurment
    }

    @Override
    public void updatePositionEstimate(GPSPoseMessage m) {
        pose = m;

    }

    //find the closest way point
    private int getClosestIndex(ArrayList<GpsMeasurement> wayPoints, GPSPoseMessage currentLocation) {
        double min = Double.MAX_VALUE; //note that the brakes will definitely deploy at this

        int closestIndex = lastClosestIndex;
        for (int i = lastClosestIndex; i < (lastClosestIndex + WAYPOINT_LOOKAHEAD_MAX) && i < wayPoints.size(); i++) {
            GPSPoseMessage gpsPoseMessage = wayPoints.get(i).toGpsPoseMessage(0);
            double d = GPSPoseMessage.getDistance(currentLocation, gpsPoseMessage);
            if (d < min) {
                min = d;
                closestIndex = i;
            }
            // eventually cut off search somehow
        }
        lastClosestIndex = closestIndex;
        return closestIndex;
    }

    @Override
    public double getCommandedSteeringAngle() {
        // determines the angle at which to move every 50 milliseconds
        //PD control of DC steering motor handled by low level
        
        double commandedAngle;
        commandedAngle = purePursuitController();
//        commandedAngle = stanleyMethodController();
//        commandedAngle = purePursuitV2();

        currentCommandedAngle = commandedAngle;
        currentDesiredHeading = pose.getHeading() + commandedAngle;
        return commandedAngle;
    }

    private double purePursuitV2() {

        // check if we should get a new waypoint
        boolean newWaypoint = false;
        // if we don't have a target then we need one
        if (target == wayPoints.get(0)) {
            newWaypoint = true;
        }
        else {
            double waypointDist = GPSPoseMessage.getDistance(pose, target.toGpsPoseMessage(0));
            // 5m is too close, pick a new one
            if (waypointDist < 5) {
                newWaypoint = true;
            }

            double currentOrientation = pose.getCurrentState().get(3, 0);
            double dx = LocalizerUtil.convertLonToMeters(target.getLongitude()) - LocalizerUtil.convertLonToMeters(pose.getLongitude());
            double dy = LocalizerUtil.convertLatToMeters(target.getLatitude()) - LocalizerUtil.convertLatToMeters(pose.getLatitude());
            double desiredHeading = Math.atan2(dy, dx);

            if (Math.abs(currentOrientation - desiredHeading) > Math.toRadians(40)){
                newWaypoint = true;
            }
        }

        // if we do, pick one which is 3 seconds away
        if (newWaypoint) {
            double lookaheadTime = 3.0;
            // pick a point between 3 & 10m
            double lookaheadLB = 3.0;
            double lookaheadUB = 10.0;
            double lookahead = pose.getCurrentState().get(2, 0) * lookaheadTime;
            if (lookahead < lookaheadLB) { lookahead = lookaheadLB; }
            if (lookahead > lookaheadUB) { lookahead = lookaheadUB; }

            int closestIndex = getClosestIndex(wayPoints, pose);
            for (int i = closestIndex; i < closestIndex + WAYPOINT_LOOKAHEAD_MAX; i++) {
                double dx = LocalizerUtil.convertLonToMeters(wayPoints.get(i).toGpsPoseMessage(0).getLongitude()) - LocalizerUtil.convertLonToMeters
                        (wayPoints.get(closestIndex).getLongitude());
                double dy = LocalizerUtil.convertLatToMeters(wayPoints.get(i).toGpsPoseMessage(0).getLatitude()) - LocalizerUtil.convertLatToMeters
                        (wayPoints.get(closestIndex).getLatitude());

                double ch = pose.getCurrentState().get(3, 0);
                double dh = Math.atan2(dy, dx);

                double theta = dh-ch;
                // pick the first point that is at least lookahead away
                if (GPSPoseMessage.getDistance(pose, wayPoints.get(i).toGpsPoseMessage(0)) * Math.cos(theta) > lookahead) {
                    target = wayPoints.get(i);
                    currentWaypoint = target;
                    break;
                }
            }
        }

        // steer towards it
        double currentOrientation = pose.getCurrentState().get(3, 0);
        double dx = LocalizerUtil.convertLonToMeters(target.getLongitude()) - LocalizerUtil.convertLonToMeters(pose.getLongitude());
        double dy = LocalizerUtil.convertLatToMeters(target.getLatitude()) - LocalizerUtil.convertLatToMeters(pose.getLatitude());
        double desiredHeading = Math.atan2(dy, dx);
        desiredHeading = Util.normalizeAngleRad(desiredHeading);

        double alpha = desiredHeading - currentOrientation;
        double L = RobobuggyKFLocalizer.WHEELBASE_IN_METERS;
        double ld = GPSPoseMessage.getDistance(target.toGpsPoseMessage(0), pose) + L / 2;
        return Math.atan2(2*L*Math.sin(alpha), ld);
    }

    private double purePursuitController() {
        // https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf
        // section 2.2

        int closestIndex = getClosestIndex(wayPoints, pose);

        double K = 2.5;
        double velocity = pose.getCurrentState().get(2, 0);
        double lookaheadLowerBound = 5.0;
        double lookaheadUpperBound = 25.0;
        double lookahead = K * velocity;
        if(lookahead < lookaheadLowerBound) {
            lookahead = lookaheadLowerBound;
        }
        else if(lookahead > lookaheadUpperBound) {
            lookahead = lookaheadUpperBound;
        }

        //pick the first point that is at least lookahead away, then point buggy toward it
        int lookaheadIndex = 0;
        for(lookaheadIndex = closestIndex; lookaheadIndex < wayPoints.size(); lookaheadIndex++) {
            if(GPSPoseMessage.getDistance(pose, wayPoints.get(lookaheadIndex).toGpsPoseMessage(0)) > lookahead) {
                break;
            }
        }

        //if we are out of points then just go straight
        if (lookaheadIndex >= wayPoints.size()) {
            new RobobuggyLogicNotification("HELP out of points", RobobuggyMessageLevel.EXCEPTION);
            return 0;
        }

        //find a path from our current location to that point
        GpsMeasurement target = wayPoints.get(lookaheadIndex);
        currentWaypoint = target;
        double dx = LocalizerUtil.convertLonToMeters(target.getLongitude()) - LocalizerUtil.convertLonToMeters(pose.getLongitude());
        double dy = LocalizerUtil.convertLatToMeters(target.getLatitude()) - LocalizerUtil.convertLatToMeters(pose.getLatitude());
        double deltaHeading = Math.atan2(dy, dx) - pose.getHeading();

        GpsMeasurement B = wayPoints.get(closestIndex);
        GpsMeasurement A = wayPoints.get(closestIndex + 1);
        GPSPoseMessage P = pose;

        double padx = LocalizerUtil.convertLonToMeters(A.getLongitude()) - LocalizerUtil.convertLonToMeters(P.getLongitude());
        double pady = LocalizerUtil.convertLatToMeters(A.getLatitude()) - LocalizerUtil.convertLatToMeters(P.getLatitude());
        double phi = Math.atan2(pady, padx);

        double badx = LocalizerUtil.convertLonToMeters(A.getLongitude()) - LocalizerUtil.convertLonToMeters(B.getLongitude());
        double bady = LocalizerUtil.convertLatToMeters(A.getLatitude()) - LocalizerUtil.convertLatToMeters(B.getLatitude());
        double psi = Math.atan2(bady, badx);

        double theta = phi - psi;

        double L = GPSPoseMessage.getDistance(P, B.toGpsPoseMessage(0));
        double E = L * Math.sin(theta);

        double thetaDelta = phi - psi;
        thetaDelta *= E/10.0;


        deltaHeading += thetaDelta;

        //Pure Pursuit steering controller
        double commandedAngle = Math.atan2(2 * RobobuggyKFLocalizer.WHEELBASE_IN_METERS * Math.sin(deltaHeading), lookahead);
        commandedAngle = Util.normalizeAngleRad(commandedAngle);
        return commandedAngle;
    }

    private double stanleyMethodController() {
        // https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf
        // section 2.3
        
        int closestIndex = getClosestIndex(wayPoints, pose);

        double K = 0.3;
        double velocity = pose.getCurrentState().get(2, 0);

        //if we are out of points then just go straight
        if (closestIndex >= (wayPoints.size() - 1)) {
            new RobobuggyLogicNotification("HELP out of points", RobobuggyMessageLevel.EXCEPTION);
            return 0;
        }

        GpsMeasurement ptA = wayPoints.get(closestIndex - 1);
        GpsMeasurement ptB = wayPoints.get(closestIndex + 1);
        double pathx = LocalizerUtil.convertLonToMeters(ptB.getLongitude()) - LocalizerUtil.convertLonToMeters(ptA.getLongitude());
        double pathy = LocalizerUtil.convertLatToMeters(ptB.getLatitude()) - LocalizerUtil.convertLatToMeters(ptA.getLatitude());
        double dx = LocalizerUtil.convertLonToMeters(pose.getLongitude()) - LocalizerUtil.convertLonToMeters(ptB.getLongitude());
        double dy = LocalizerUtil.convertLatToMeters(pose.getLatitude()) - LocalizerUtil.convertLatToMeters(ptB.getLatitude());
        currentWaypoint = wayPoints.get(closestIndex);

        double pathHeading = Math.atan2(pathy, pathx);
        double headingError = Util.normalizeAngleRad(pathHeading) - Util.normalizeAngleRad(pose.getHeading());
        double commandedAngle;
        double L = GPSPoseMessage.getDistance(pose, ptB.toGpsPoseMessage(0));
        double theta = Math.atan2(dy, dx);

        double crosstrackError = L * Math.sin(theta);

        //Stanley steering controller
        commandedAngle = headingError + Math.atan2(K * crosstrackError, velocity);
        commandedAngle = Util.normalizeAngleRad(commandedAngle);
        return commandedAngle;
    }

    @Override
    protected boolean getDeployBrakeValue() {
        // need to brake when 15 m off course
        return false;
    }


}
