package com.roboclub.robobuggy.nodes.localizers;

import Jama.Matrix;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.messages.IMUCompassMessage;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyBaseNode;
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
    private Matrix covariance;            // a matrix that defines our levels of trust for each sensor
    private Matrix currentStatePose;      // our current position estimate
    private Matrix currentStateNoise;     // our current noise estimate
    private Matrix nextStatePose;         // an estimate of the next state, based on the motion model
    private Matrix nextStateNoise;        // an estimate of the next state's noise, based on the motion model
    private Matrix motionModel;           // a set of equations that model how the buggy behaves

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
    protected RobobuggyKFLocalizer(int period, String name, LocTuple initialPosition) {
        super(new BuggyBaseNode(NodeChannel.POSE), period, name);
        posePub = new Publisher(NodeChannel.POSE.getMsgPath());
        this.initialPosition = initialPosition;

        // set our initial covariance
        // The covariance matrix consists of 7 rows:
        //      x       - x position in the world frame, in meters
        //      y       - y position in the world frame, in meters
        //      x_b     - the body velocity of the buggy (x-velocity relative to the buggy frame)
        //      y_b     - y velocity relative to the buggy frame (always 0 since we cant strafe)
        //      th      - yaw angle with respect to the world frame
        //      th_dot  - change in th over time
        //      heading - current steering angle
        covariance = new Matrix(SEVEN_ROW_IDENTITY_MATRIX);

        // set our initial state
        //  - take our initial position and perform UTM conversions on it
        // todo set the initial state


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
        // a kalman filter consists of two distinct steps - predict and update
        /*

            <x[k-1], p[k-1]> ---> {A} ---> <xhat[k], phat[k]> ---> |
                   ^                                               |> {filter} ---> <x[k], p[k]> ---|
                   |                                        z ---> |                                |
                   |                                                                                |
                   ---------------------------------------------------------------------------------|

         */

        // the predict step is responsible for determining the estimate of the next state
        //
        //   What we do is we take our current state (x[k-1]) and current noise estimate
        //   (p[k-1]), and plug them into the motion model (A). This lets us determine our
        //   prediction state (xhat[k]) and our prediction noise (phat[k])
        //
        predictStep();

        // the update step is responsible for updating the current state, and resolving
        // discrepancies between the prediction and actual state
        //
        //   What we do is we take our measurements (z) that were captured from sensors,
        //   as well as the predictions from the previous step (xhat[k], phat[k]) and run
        //   them through the filter which todo fill this part out about innovation etc
        //
        //   This produces your next state (x[k]) and your next noise estimation (p[k]),
        //   which connect in a feedback loop to become the new current state
        //
        updateStep();

        // publish the current findings

        // and update our understanding of time
        currentStatePose = nextStatePose;
        currentStateNoise = nextStateNoise;
    }

    private void predictStep() {

    }

    private void updateStep() {

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
