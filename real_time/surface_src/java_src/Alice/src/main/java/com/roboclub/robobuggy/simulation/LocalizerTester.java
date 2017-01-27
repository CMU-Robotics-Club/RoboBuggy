package com.roboclub.robobuggy.simulation;

import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyBaseNode;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyDecoratorNode;
import com.roboclub.robobuggy.nodes.localizers.LocTuple;
import com.roboclub.robobuggy.ros.NodeChannel;

import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;

/**
 * Created by vivaanbahl on 1/27/17.
 */
public class LocalizerTester extends BuggyDecoratorNode {

    private static final int GPS_UPDATE_PERIOD = 500;
    private static final int ODOM_UPDATE_PERIOD = 50;
    private static final double POSITION_UPDATE_M = 0.5; // move this many meters every tick

    private GpsMeasurement targetWaypoint;
    private Timer gpsTimer;
    private Timer odomTimer;
    private LocTuple currentPosition;
    private double heading;
    private double noise;

    /**
     * Creates a new decorator for the given {@link Node}
     *
     * @param node {@link Node} to decorate
     * @param name the name we want for this node to store so that it can be referenced later
     */
    public LocalizerTester(String name, ArrayList<GpsMeasurement> waypoints, double noise) {
        super(new BuggyBaseNode(NodeChannel.POSE), name);

        gpsTimer = new Timer("GPS");
        odomTimer = new Timer("odom");
        this.noise = noise;
    }

    @Override
    protected boolean startDecoratorNode() {
        gpsTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                //todo update sim gps
            }
        }, 0, GPS_UPDATE_PERIOD);

        odomTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                // todo update sim odom
            }
        }, 0, ODOM_UPDATE_PERIOD);

        return true;
    }

    @Override
    protected boolean shutdownDecoratorNode() {
        return false;
    }
}
