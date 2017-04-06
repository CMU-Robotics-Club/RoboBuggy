package com.roboclub.robobuggy.nodes.planners;

import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.ros.NodeChannel;

import java.util.ArrayList;
import java.util.Date;

/**
 * Created by vivaanbahl on 4/4/17.
 */
public class PolynomialPlanner extends PathPlannerNode {

    private ArrayList<GpsMeasurement> waypoints;
    private Date expirationTime;
    private double previousSteeringAngle;
    private GPSPoseMessage currentPose;
    private GpsMeasurement target;
    private ArrayList<GpsMeasurement> checkInPoints;

    // switch checkin points if we are 2m away from the current target checkin
    private static final int SWITCHING_TOLERANCE_M = 2;
    // recompute the polynomial every 1000ms
    private static final int POLYNOMIAL_RECOMPUTE_PERIOD = 1000;
    // sample the polynomial to get checkin points every 50 ms along the curve
    private static final int CHECKIN_SAMPLE_PERIOD = 50;

    /**
     * Construct a new {@link PathPlannerNode}
     *
     */
    public PolynomialPlanner(ArrayList<GpsMeasurement> waypoints) {
        super(NodeChannel.PATH_PLANNER);
        this.waypoints = waypoints;
        previousSteeringAngle = 0;
        expirationTime = new Date();
    }

    @Override
    protected void updatePositionEstimate(GPSPoseMessage m) {

        currentPose = m;

    }

    @Override
    protected double getCommandedSteeringAngle() {
        Date currentTime = new Date();
        long elapsedMilliseconds = expirationTime.getTime() - currentTime.getTime();

        // if we've expired the current polynomial curve either in time or in space recompute
        if (elapsedMilliseconds < 0 || checkInPoints.size() == 0) {

            ArrayList<GPSPoseMessage> samplePoints = new ArrayList<>();
            samplePoints.add(currentPose);

            // get all the waypoints within the recompute period away from us (velocity times period)
            double lookaheadDist = currentPose.getCurrentState().get(2, 0) * POLYNOMIAL_RECOMPUTE_PERIOD;
            int lookaheadIndex = 0;
            for (GpsMeasurement candidate : waypoints) {
                if (GPSPoseMessage.getDistance(currentPose, candidate.toGpsPoseMessage(0)) < lookaheadDist) {
                    samplePoints.add(candidate.toGpsPoseMessage(0));
                }
            }

            // build our regression fit matrix
            double[] yDouble = new double[samplePoints.size()];
            double[][] xDouble = new double[samplePoints.size()][samplePoints.size()];
            // TODO: 4/6/17 fill this in tomorrow

        }
        // otherwise we still should be following this curve, check whether we're too close to this current check-in point
        else {
            GPSPoseMessage targetAsPose = target.toGpsPoseMessage(0);
            double targetDistance = GPSPoseMessage.getDistance(currentPose, targetAsPose);
            if (targetDistance <= SWITCHING_TOLERANCE_M) {
                target = checkInPoints.remove(0);
            }
        }

        return 0;
    }

    @Override
    protected boolean getDeployBrakeValue() {
        return false;
    }
}
