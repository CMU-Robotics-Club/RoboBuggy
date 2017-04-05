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
        
        double commandedAngle = purePursuitController();
//        double commandedAngle = stanleyMethodController();
        
        currentCommandedAngle = commandedAngle;
        currentDesiredHeading = pose.getHeading() + commandedAngle;
        return commandedAngle;
    }

    private double purePursuitController() {
        // https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf
        // section 2.2

        int closestIndex = getClosestIndex(wayPoints, pose);

        double K = 3.0;
        double velocity = pose.getCurrentState().get(2, 0);
        double lookaheadLowerBound = 3.0;
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

        //Pure Pursuit steering controller
        double commandedAngle = Math.atan2(2 * RobobuggyKFLocalizer.WHEELBASE_IN_METERS * Math.sin(deltaHeading), lookahead);
        commandedAngle = Util.normalizeAngleRad(commandedAngle);
        return commandedAngle;
    }

    private double stanleyMethodController() {
        // https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf
        // section 2.3
        
        int closestIndex = getClosestIndex(wayPoints, pose);

        double K = 0.1;
        double velocity = pose.getCurrentState().get(2, 0);

        //if we are out of points then just go straight
        if (closestIndex >= (wayPoints.size() - 1)) {
            new RobobuggyLogicNotification("HELP out of points", RobobuggyMessageLevel.EXCEPTION);
            return 0;
        }

        GpsMeasurement ptA = wayPoints.get(closestIndex);
        GpsMeasurement ptB = wayPoints.get(closestIndex + 1);
        double pathx = LocalizerUtil.convertLonToMeters(ptB.getLongitude()) - LocalizerUtil.convertLonToMeters(ptA.getLongitude());
        double pathy = LocalizerUtil.convertLatToMeters(ptB.getLatitude()) - LocalizerUtil.convertLatToMeters(ptA.getLatitude());
        double dx = LocalizerUtil.convertLonToMeters(pose.getLongitude()) - LocalizerUtil.convertLonToMeters(ptA.getLongitude());
        double dy = LocalizerUtil.convertLatToMeters(pose.getLatitude()) - LocalizerUtil.convertLatToMeters(ptA.getLatitude());

        double pathHeading = Math.atan2(pathy, pathx);
        double headingError = Util.normalizeAngleRad(pathHeading) - Util.normalizeAngleRad(pose.getHeading());
        double determinant = (pathx * dy) - (pathy - dx);
        double crosstrackError = - determinant / Math.sqrt(pathx*pathx + pathy*pathy);

        //Stanley steering controller
        double commandedAngle = headingError + Math.atan2(K * crosstrackError, velocity);
        commandedAngle = Util.normalizeAngleRad(commandedAngle);
        return commandedAngle;
    }

    @Override
    protected boolean getDeployBrakeValue() {
        // need to brake when 15 m off course
        return false;
    }


}
