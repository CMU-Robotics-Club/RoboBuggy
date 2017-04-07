package com.roboclub.robobuggy.nodes.planners;

import Jama.Matrix;
import com.roboclub.robobuggy.main.Util;
import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.nodes.localizers.LocalizerUtil;
import com.roboclub.robobuggy.nodes.localizers.RobobuggyKFLocalizer;
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
    // we want 20 points sampled from the polynomial
    private static final int CHECKIN_SAMPLE_SIZE = 20;
    // we want our points 2m away from each other (laterally)
    private static final double CHECKIN_POINT_SAMPLE_DISTANCE = LocalizerUtil.convertMetersToLon(2);
    // quadratic interpolation, need 3 terms (including x^0)
    private static final int POLYNOMIAL_NUMBER_OF_TERMS = 3;

    /**
     * Construct a new {@link PathPlannerNode}
     *
     */
    public PolynomialPlanner(ArrayList<GpsMeasurement> waypoints) {
        super(NodeChannel.PATH_PLANNER);
        this.waypoints = waypoints;
        previousSteeringAngle = 0;
        expirationTime = new Date();
        checkInPoints = new ArrayList<>();
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
            double lookaheadDist = currentPose.getCurrentState().get(2, 0) * POLYNOMIAL_RECOMPUTE_PERIOD / 1000.0;
            for (GpsMeasurement candidate : waypoints) {
                if (GPSPoseMessage.getDistance(currentPose, candidate.toGpsPoseMessage(0)) < lookaheadDist) {
                    samplePoints.add(candidate.toGpsPoseMessage(0));
                }
            }

            // build our regression fit matrix
            double[][] yDouble = new double[POLYNOMIAL_NUMBER_OF_TERMS][1];
            double[][] xDouble = new double[POLYNOMIAL_NUMBER_OF_TERMS][POLYNOMIAL_NUMBER_OF_TERMS];
            for (int i = 0; i < POLYNOMIAL_NUMBER_OF_TERMS; i++) {

                for (GPSPoseMessage samplePoint : samplePoints) {
                    yDouble[i][0] += samplePoint.getLatitude() * Math.pow(samplePoint.getLongitude(), i);
                }

                for (int j = 0; j < POLYNOMIAL_NUMBER_OF_TERMS; j++) {
                    for (GPSPoseMessage samplePoint : samplePoints) {
                        xDouble[i][j] += Math.pow(samplePoint.getLongitude(), i + j);
                    }
                }
            }
            // since y will be horizontal but we want a vertical vector
            Matrix y = new Matrix(yDouble);
            Matrix x = new Matrix(xDouble);

            // get the coefficients of the polynomial
            Matrix a = x.inverse().times(y);

            // build up our check in array
            checkInPoints = new ArrayList<>();

            for (int i = -CHECKIN_SAMPLE_SIZE; i < CHECKIN_SAMPLE_SIZE; i++) {
                double checkin_x = currentPose.getLongitude() + i*CHECKIN_POINT_SAMPLE_DISTANCE;
                double checkin_y = 0;
                for (int j = 0; j < POLYNOMIAL_NUMBER_OF_TERMS; j++) {
                    checkin_y += a.get(j, 0) * Math.pow(checkin_x, j);
                }

                checkInPoints.add(new GpsMeasurement(checkin_x, checkin_y));
            }

        }

        // otherwise we still should be following this curve, get the closest checkin point
        target = checkInPoints.get(0);
        double minDistance = Double.MAX_VALUE;
        for (GpsMeasurement candidate: checkInPoints) {
            GPSPoseMessage candidatePose = candidate.toGpsPoseMessage(0);

            // figure out heading diff between current and candidate, if it's "behind" (> 45deg), skip
            double buggyOrientation = currentPose.getHeading();
            double checkpointDx = LocalizerUtil.convertLonToMeters(currentPose.getLongitude()) - LocalizerUtil.convertLonToMeters(candidatePose.getLongitude());
            double checkpointDy = LocalizerUtil.convertLatToMeters(currentPose.getLatitude()) - LocalizerUtil.convertLatToMeters(candidatePose.getLatitude());
            double checkpointOrientation = Math.atan2(checkpointDy, checkpointDx);

            double deltaHeading = Math.abs(buggyOrientation - checkpointOrientation);
            if (deltaHeading < Math.toRadians(45)) {
                double candidateDistance = GPSPoseMessage.getDistance(currentPose, candidatePose);
                if (candidateDistance < minDistance) {
                    minDistance = candidateDistance;
                    target = candidate;
                }
            }

        }


        // steer towards the target
        double commandedAngle = 0;
        double dx = LocalizerUtil.convertLonToMeters(target.getLongitude()) - LocalizerUtil.convertLonToMeters(currentPose.getLongitude());
        double dy = LocalizerUtil.convertLatToMeters(target.getLatitude()) - LocalizerUtil.convertLatToMeters(currentPose.getLatitude());
        double deltaHeading = Math.atan2(dy, dx) - currentPose.getHeading();
        commandedAngle = Math.atan2(2 * RobobuggyKFLocalizer.WHEELBASE_IN_METERS * Math.sin(deltaHeading), minDistance);
        commandedAngle = Util.normalizeAngleRad(commandedAngle);

        return commandedAngle;
    }

    @Override
    protected boolean getDeployBrakeValue() {
        return false;
    }
}
