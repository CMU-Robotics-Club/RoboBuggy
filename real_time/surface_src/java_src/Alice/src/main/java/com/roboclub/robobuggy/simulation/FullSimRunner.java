package com.roboclub.robobuggy.simulation;

import Jama.Matrix;
import com.roboclub.robobuggy.main.RobobuggyConfigFile;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyBaseNode;
import com.roboclub.robobuggy.nodes.baseNodes.PeriodicNode;
import com.roboclub.robobuggy.nodes.localizers.LocTuple;
import com.roboclub.robobuggy.nodes.localizers.LocalizerUtil;
import com.roboclub.robobuggy.nodes.localizers.RobobuggyKFLocalizer;
import com.roboclub.robobuggy.nodes.localizers.UTMTuple;
import com.roboclub.robobuggy.nodes.planners.WayPointFollowerPlanner;
import com.roboclub.robobuggy.nodes.planners.WayPointUtil;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.Subscriber;

import java.io.FileNotFoundException;
import java.util.Timer;
import java.util.TimerTask;

/**
 * Created by vivaanbahl on 4/2/17.
 */
public class FullSimRunner extends PeriodicNode {

    private static final double WHEELBASE_IN_METERS = 1.13; // meters
    private static final int UTMZONE = 17;
    private static final int VELOCITY = 3;
    private static final double INITIAL_HEADING_IN_RADS = 4.36; // rad
    private static final int LOCALIZER_UPDATE_PERIOD = 10;
    private static final int CONTROLLER_UPDATE_PERIOD = 10;
    private static final int GPS_UPDATE_PERIOD = 500;
    private static final int ENC_UPDATE_PERIOD = 10;

    private Matrix motionModel;
    private Matrix state;

    private Publisher gpsPub;
    private Publisher encPub;
    private Subscriber steerSub;

    private RobobuggyKFLocalizer localizer;
    private WayPointFollowerPlanner controller;

    private Timer gpsTimer;
    private Timer encTimer;

    /**
     * Create a new {@link PeriodicNode} decorator
     *
     * @param period of the periodically executed portion of the node
     * @param name
     */
    protected FullSimRunner(int period, String name, LocTuple initialPos) {
        super(new BuggyBaseNode(NodeChannel.SIMULATION), period, name);

        // initialized to the identity matrix (goes nowhere)
        double[][] motionModelArr = {
                {1, 0, 0, 0, 0},
                {0, 1, 0, 0, 0},
                {0, 0, 1, 0, 0},
                {0, 0, 0, 1, 0},
                {0, 0, 0, 0, 1},
        };
        motionModel = new Matrix(motionModelArr);

        UTMTuple t = LocalizerUtil.deg2UTM(initialPos);
        double[][] stateArr = {
                { t.getEasting() },
                { t.getNorthing() },
                { VELOCITY },
                { INITIAL_HEADING_IN_RADS },
                { 0 },
        };
        state = new Matrix(stateArr);

        gpsPub = new Publisher(NodeChannel.GPS.getMsgPath());
        encPub = new Publisher(NodeChannel.ENCODER.getMsgPath());
        steerSub = new Subscriber("Full Sim Toolbox", NodeChannel.STEERING.getMsgPath(), (topicName, m) -> {
            updateMotionModelSteering(Math.toRadians(((SteeringMeasurement) m).getAngle()));
        });

        try {
            localizer = new RobobuggyKFLocalizer(LOCALIZER_UPDATE_PERIOD, "Simulated Localization", initialPos);
            controller = new WayPointFollowerPlanner(WayPointUtil.createWayPointsFromWaypointList(RobobuggyConfigFile.getWaypointSourceLogFile()));
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

        gpsTimer = new Timer();
        gpsTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                // TODO: 4/3/17 fill this out once we get the main loop running
            }
        }, 0, GPS_UPDATE_PERIOD);

        encTimer = new Timer();
        encTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                // TODO: 4/3/17 fill this out once we get the main loop running
            }
        }, 0, ENC_UPDATE_PERIOD);

    }

    private void updateMotionModelSteering(double v) {
        // TODO: 4/3/17 fill this out once we get the main loop running
    }

    @Override
    protected void update() {

    }

    @Override
    protected boolean startDecoratorNode() {
        return false;
    }

    @Override
    protected boolean shutdownDecoratorNode() {
        return false;
    }
}
