package com.roboclub.robobuggy.nodes.localizers;

import Jama.Matrix;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.messages.IMUCompassMessage;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyNode;
import com.roboclub.robobuggy.nodes.baseNodes.PeriodicNode;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.Subscriber;

import java.util.Date;

/**
 * Created by vivaanbahl on 11/4/16.
 */
public class RobobuggyKFLocalizer extends PeriodicNode {

    // our transmitter for position estimate messages
    private Publisher posePub;

    // matrices that are used in the kalman filter
    private Matrix covariance;
    private Matrix state;
    private Matrix motionMatrix;

    // state variables
    private LocTuple currentStateGPS;     // current GPS location
    private double currentStateEncoder;   // current deadreckoning value
    private double currentStateHeading;   // current heading in degrees
    private long currentStateEncoderTime; // current encoder time
    private Date currentStateTime;        // current time

    // constants we use throughout the file
    private final LocTuple initialPosition;
    private static final double WHEELBASE = 1.13; // in meters
    private static final double[][] SEVEN_ROW_IDENTITY_MATRIX = {
            {1, 0, 0, 0, 0, 0, 0},
            {0, 1, 0, 0, 0, 0, 0},
            {0, 0, 1, 0, 0, 0, 0},
            {0, 0, 0, 1, 0, 0, 0},
            {0, 0, 0, 0, 1, 0, 0},
            {0, 0, 0, 0, 0, 1, 0},
            {0, 0, 0, 0, 0, 0, 1}
    };



    /**
     * Create a new {@link PeriodicNode} decorator
     *
     * @param base   {@link BuggyNode} to decorate
     * @param period of the periodically executed portion of the node
     * @param name the name of the node
     * @param initialPosition the initial position of the localizer
     */
    protected RobobuggyKFLocalizer(BuggyNode base, int period, String name, LocTuple initialPosition) {
        super(base, period, name);
        posePub = new Publisher(NodeChannel.POSE.getMsgPath());
        this.initialPosition = initialPosition;

        // set our initial covariance
        // turns out it's just the I matrix
        // todo fill out descriptions
        // The covariance matrix consists of 7 rows:
        //      x       - x position in the world frame, in meters
        //      y       - y position in the world frame, in meters
        //      x_b     -
        //      y_b     -
        //      th      -
        //      th_dot  -
        //      heading -
        covariance = new Matrix(SEVEN_ROW_IDENTITY_MATRIX);

        // set our initial state
        //  - take our initial position and perform UTM conversions on it


        // add all our subscribers for our current state update stream
        // these subscribers are only meant as catchers, just stupidly simple relays
        // that feed the current state variables
        setupGPSSubscriber();
        setupIMUSubscriber();
        setupEncoderSubscriber();
    }

    private void setupEncoderSubscriber() {
        new Subscriber("KF Localizer", NodeChannel.ENCODER.getMsgPath(), ((topicName, m) -> {
            EncoderMeasurement deadreckoning = ((EncoderMeasurement) m);
            currentStateEncoder = deadreckoning.getDistance();
        }));
    }

    private void setupIMUSubscriber() {
        new Subscriber("KF Localizer", NodeChannel.IMU_COMPASS.getMsgPath(), ((topicName, m) -> {
            IMUCompassMessage compassMessage = ((IMUCompassMessage) m);
            currentStateHeading = compassMessage.getCompassHeading();
        }));
    }

    private void setupGPSSubscriber() {
        new Subscriber("KF Localizer", NodeChannel.GPS.getMsgPath(), ((topicName, m) -> {
            GpsMeasurement gpsLoc = ((GpsMeasurement) m);
            currentStateGPS = new LocTuple(gpsLoc.getLatitude(), gpsLoc.getLongitude());
        }));
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
