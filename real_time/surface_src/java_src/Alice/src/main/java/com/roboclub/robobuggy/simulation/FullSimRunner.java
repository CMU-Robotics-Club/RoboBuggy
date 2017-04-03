package com.roboclub.robobuggy.simulation;

import Jama.Matrix;
import com.roboclub.robobuggy.main.RobobuggyConfigFile;
import com.roboclub.robobuggy.messages.DriveControlMessage;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.GpsMeasurement;
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
import java.util.Date;
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
    private static final int GPS_UPDATE_PERIOD = 500;
    private static final int ENC_UPDATE_PERIOD = 100;
    private static final int SIM_UPDATE_PERIOD = 100;

    private Matrix motionModel;
    private Matrix state;
    private double totalDistance = 0;

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
     * @param name
     */
    public FullSimRunner(String name, LocTuple initialPos) {
        super(new BuggyBaseNode(NodeChannel.SIMULATION), SIM_UPDATE_PERIOD, name);

        // initialized to the identity matrix except for dHeading (goes nowhere)
        double[][] motionModelArr = {
                {1, 0, 0, 0, 0},
                {0, 1, 0, 0, 0},
                {0, 0, 1, 0, 0},
                {0, 0, 0, 1, 0},
                {0, 0, 0, 0, 0},
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
        steerSub = new Subscriber("Full Sim Toolbox", NodeChannel.DRIVE_CTRL.getMsgPath(), (topicName, m) -> {
            updateMotionModel(((DriveControlMessage) m).getAngleDouble());
        });
        updateMotionModel(0);

        try {
            localizer = new RobobuggyKFLocalizer(LOCALIZER_UPDATE_PERIOD, "Simulated Localization", initialPos);
            localizer.startNode();
            controller = new WayPointFollowerPlanner(WayPointUtil.createWayPointsFromWaypointList(RobobuggyConfigFile.getWaypointSourceLogFile()));
            controller.startNode();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

        gpsTimer = new Timer();
        gpsTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                Matrix nextStep = generateNextStep();
                UTMTuple nextLocUTM = new UTMTuple(UTMZONE, 'T', state.get(0, 0), state.get(1, 0));
                LocTuple nextLocLL = LocalizerUtil.utm2Deg(nextLocUTM);
                gpsPub.publish(new GpsMeasurement(new Date(), nextLocLL.getLatitude(), true, nextLocLL.getLongitude(), true, 0, 0, 0, 0, 0, 0));
                state = nextStep;

            }
        }, 0, GPS_UPDATE_PERIOD);

        encTimer = new Timer();
        encTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                state = generateNextStep();
                double deltaDist = ENC_UPDATE_PERIOD / 1000.0 * state.get(2, 0);
                totalDistance += deltaDist;
                encPub.publish(new EncoderMeasurement(totalDistance, state.get(2, 0)));
            }
        }, 0, ENC_UPDATE_PERIOD);

    }

    private void updateMotionModel(double v) {
        double dt = ENC_UPDATE_PERIOD / 1000.0;
        motionModel.set(0, 2, dt*Math.cos(state.get(3, 0)));
        motionModel.set(1, 2, dt*Math.sin(state.get(3, 0)));
        motionModel.set(3, 4, dt);
        motionModel.set(4, 2, Math.tan(v) / WHEELBASE_IN_METERS);

    }

    private Matrix generateNextStep() {
        return motionModel.times(state);
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
