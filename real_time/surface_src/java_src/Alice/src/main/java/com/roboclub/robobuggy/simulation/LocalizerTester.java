package com.roboclub.robobuggy.simulation;

import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyBaseNode;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyDecoratorNode;
import com.roboclub.robobuggy.nodes.localizers.LocTuple;
import com.roboclub.robobuggy.nodes.localizers.LocalizerUtil;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;

import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;

/**
 * Created by vivaanbahl on 1/27/17.
 */
public class LocalizerTester extends BuggyDecoratorNode {

    private static final int GPS_UPDATE_PERIOD = 500;
    private static final int ODOM_UPDATE_PERIOD = 50;
    private static final double POSITION_UPDATE_M = 1; // move this many meters every tick

    private int targetWaypointIndex = 0;
    private ArrayList<GpsMeasurement> waypoints;
    private Timer gpsTimer;
    private Timer odomTimer;
    private LocTuple currentPosition = new LocTuple(40.441670, -79.9416362);
    private double heading = Math.toRadians(90);
    private double noise; // todo insert noise

    private Publisher gpsPub = new Publisher(NodeChannel.GPS.getMsgPath());
    private Publisher odomPub = new Publisher(NodeChannel.ENCODER.getMsgPath());

    /**
     * Creates a new decorator for the given Node
     *
     * @param name the name we want for this node to store so that it can be referenced later
     */
    public LocalizerTester(String name, ArrayList<GpsMeasurement> waypoints, double noise) {
        super(new BuggyBaseNode(NodeChannel.POSE), name);

        gpsTimer = new Timer("GPS");
        odomTimer = new Timer("odom");
        this.noise = noise;
        this.waypoints = waypoints;
    }

    public GpsMeasurement getTargetWaypoint() {
        GpsMeasurement currentTarget = waypoints.get(targetWaypointIndex);
        double distInM = (GpsMeasurement.getDistance(currentTarget, new GpsMeasurement(currentPosition.getLatitude(),
                currentPosition
                .getLongitude())));
        if (distInM < 5) {
            targetWaypointIndex++;
        }
        return waypoints.get(targetWaypointIndex);
    }

    @Override
    protected boolean startDecoratorNode() {
        gpsTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                GpsMeasurement targetWaypoint = getTargetWaypoint();
                double dlat = currentPosition.getLatitude() - targetWaypoint.getLatitude();
                double dlon = currentPosition.getLongitude() - targetWaypoint.getLongitude();
                heading = Math.atan2(dlat, -dlon) + Math.toRadians(90);

                double updateLat = LocalizerUtil.convertMetersToLat(POSITION_UPDATE_M) * Math.cos(heading);
                double updateLon = LocalizerUtil.convertMetersToLat(POSITION_UPDATE_M) * Math.sin(heading);
                double newLat = currentPosition.getLatitude() + updateLat;
                double newLon = currentPosition.getLongitude() + updateLon;
                currentPosition = new LocTuple(newLat, newLon);
                gpsPub.publish(new GpsMeasurement(newLat, newLon));
            }
        }, 0, GPS_UPDATE_PERIOD);

//        odomTimer.scheduleAtFixedRate(new TimerTask() {
//            @Override
//            public void run() {
//                // todo update sim odom
//            }
//        }, 0, ODOM_UPDATE_PERIOD);

        return true;
    }

    @Override
    protected boolean shutdownDecoratorNode() {
        return false;
    }
}
