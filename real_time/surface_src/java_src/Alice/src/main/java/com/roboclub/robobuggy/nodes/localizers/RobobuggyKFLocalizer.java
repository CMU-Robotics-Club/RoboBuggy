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

    // constants we use throughout the file
    private static final double WHEELBASE = 1.13; // meters
    private static final int UTMZONE = 17;
    private final LocTuple initialGPS = LocalizerUtil.deg2UTM(
                                            new LocTuple(40.441670, -79.9416362));
    private final double initialHeading = 4.36; // rad

    // The state consists of 4 elements:
    //      x        - x position in the world frame, in meters
    //      y        - y position in the world frame, in meters
    //      dy_body  - forward velocity in the body frame, in meters/s
    //      heading  - heading in the world frame, in rad
    //      dheading - angular velocity in the world frame, in rad/s
    private Matrix R = arrayToMatrix({4, 4, 0.25, 0.01, 0.01});   // measurement noise covariance matrix
    private Matrix Q = arrayToMatrix({4, 4, 0.25, 0.02, 0.02});   // model noise covariance matrix
    private Matrix P = arrayToMatrix({25, 25, 0.25, 2.46, 2.46}); // covariance matrix
    private Matrix x; // state
    // output matrices
    private Matrix C_gps = new Matrix({
        {1, 0, 0, 0, 0},
        {0, 1, 0, 0, 0},
        {0, 0, 0, 1, 0}
    });
    private Matrix C_encoder = new Matrix({
        {0, 0, 1, 0, 0}
    });

    private Date lastTime;
    private UTMTuple lastGPS;
    // private Date lastGPSTime;
    private double lastEncoder; // deadreckoning value
    private long lastEncoderTime;
    private double currentEncoder;
    private long currentEncoderTime;
    private double steeringAngle = 0;

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

        // set initial state
        lastTime = new Date().getTime();
        lastEncoder = 0;
        lastEncoderTime = lastTime;
        lastGPS = initialGPS;
        // lastGPSTime = lastTime;

        x = new Matrix({
            { initialGPS.getEasting() },
            { initialGPS.getNorthing() },
            { 0 },
            { initialHeading },
            { 0 }
        });


        // add all our subscribers for our current state update stream
        // these subscribers are only meant as catchers, just stupidly simple relays
        // that feed the current state variables
        setupGPSSubscriber();
        // setupIMUSubscriber();
        setupEncoderSubscriber();
        setupWheelSubscriber();
    }

    private void setupEncoderSubscriber() {
        new Subscriber("KF Localizer", NodeChannel.ENCODER.getMsgPath(), ((topicName, m) -> {
            long currentTime = new Date().getTime();
            long dt = currentTime - lastEncoderTime;
            // to remove numeric instability, limit rate to 10ms, 100Hz
            if (dt < 10) {
                return;
            }

            EncoderMeasurement odometry = (EncoderMeasurement) m;
            double currentEncoder = odometry.getDistance();

            double dx = currentEncoder - lastEncoder;
            double bodySpeed = dx / (dt / 1000.0);
            lastEncoderTime = currentTime;
            lastEncoder = currentEncoder;
            
            // measurement
            Matrix z = new Matrix({
                { bodySpeed }
            });

            kalmanFilter(C_encoder, z);
        }));
    }

    // private void setupIMUSubscriber() {
    //     new Subscriber("KF Localizer", NodeChannel.IMU_COMPASS.getMsgPath(), ((topicName, m) -> {
    //         IMUCompassMessage compassMessage = (IMUCompassMessage) m;
    //         currentHeading = compassMessage.getCompassHeading();
    //     }));
    // }

    private void setupGPSSubscriber() {
        new Subscriber("KF Localizer", NodeChannel.GPS.getMsgPath(), ((topicName, m) -> {
            GpsMeasurement gpsLoc = (GpsMeasurement) m;
            LocTuple loc = new LocTuple(gpsLoc.getLatitude(), gpsLoc.getLongitude());
            UTMTuple gps = LocalizerUtil.deg2UTM(loc);
            double dx = gps.getEasting() - lastGPS.getEasting();
            double dy = gps.getNorthing() - lastGPS.getNorthing();
            lastGPS = gpsUTM;

            // don't update angle if we did not move a lot
            double heading = Math.atan2(dy, dx);
            if ((dx * dx + dy * dy) < 0.25) {
                heading = x.get(3, 0);
            }
            // close the loop
            if (Math.abs(gps.getEasting() - initialGPS.getEasting())
                    + Math.abs(gps.getNorthing() - initialGPS.getNorthing()) < 10.0) {
                heading = initialHeading;
            }

            // measurement
            Matrix z = new Matrix({
                { gps.getEasting() },
                { gps.getNorthing() },
                { heading }
            });

            kalmanFilter(C_gps, z);
        }));
    }

    private void setupWheelSubscriber() {
        new Subscriber("htGpsLoc", NodeChannel.STEERING.getMsgPath(), ((topicName, m) -> {
            SteeringMeasurement steerM = (SteeringMeasurement) m;
            steeringAngle = steerM.getAngle();
        }));
    }

    @Override
    protected void update() {
        Matrix x_predict = propagate();

        UTMTuple utm = new UTMTuple(UTMZONE, 'T', x_predict.get(0, 0),
                x_predict.get(1, 0));
        LocTuple latLon = LocalizerUtil.utm2Deg(utm);
        posePub.publish(new GPSPoseMessage(new Date(), latLon.getLatitude(),
                latLon.getLongitude(), x_predict.get(4, 0)));
    }

    // Kalman filter step 0: Generate the motion model for the buggy
    private Matrix motionModel(double dt) {
        return new Matrix({
                // x y dy_b heading dheading
                { 1, 0, dt * Math.cos(x.get(4, 0)), 0, 0 }, // x
                { 0, 1, dt * Math.sin(x.get(4, 0)), 0, 0 }, // y
                { 0, 0, 1, 0, 0 }, // dy_b
                { 0, 0, 0, 1, dt, 0 }, // heading
                { 0, 0, Math.tan(steeringAngle) / WHEELBASE, 0, 0 }, // dheading
        });
    }

    // Kalman filter step 1: Use the motion model to predict the next state 
    // and covariance matrix. (_pre = predicted)
    // Kalman filter step 2: Update the prediction based off of measurements /
    // sensor readings
    private void kalmanFilter(Matrix C, Matrix z) {
        // update time
        Date now = new Date();
        double dt = (now.getTime() - lastTime) / 1000.0;
        lastTime = now.getTime();

        Matrix A = motionModel(dt);

        /*
        Predict:
            x_pre = A * x
            P_pre = A * P * A' + R
        */
        Matrix x_pre = A.times(x);
        Matrix P_pre = A.times(P).times(A.transpose());
        P_pre = P_pre.plus(R);
        
        x_pre.set(3, 0, scrubAngle(x_pre.get(3, 0)));
        x_pre.set(4, 0, scrubAngle(x_pre.get(4, 0)));

        /* 
        Update:
            r = z - (C * x_pre)
            K = P_pre * C' * inv((C * P_pre * C') + Q) // gain
            x = x_pre + (K * r)
            P = (I - (K * C)) * P_pre
        */
        Matrix residual = z.minus(C.times(x));
        Matrix K = C.times(P_pre).times(C.transpose()).plus(Q);
        K = P_pre.times(C.transpose()).times(K.inverse());
        x = x_pre.plus(K.times(residual));
        P = Matrix.identity(5, 5).minus(K.times(C));
        P = P.times(P_pre);

        x.set(3, 0, scrubAngle(x.get(3, 0)));
        x.set(4, 0, scrubAngle(x.get(4, 0)));
    }

    // Propagate the motion model forward in time since the last sensor reading
    // not part of the Kalman algorithm
    private Matrix propagate() {
        Date now = new Date();
        double dt = (now.getTime() - lastTime) / 1000.0;

        // x_pre = A * x
        Matrix A = motionModel(dt);
        return A.times(x);
    }


    @Override
    protected boolean startDecoratorNode() {
        return false;
    }

    @Override
    protected boolean shutdownDecoratorNode() {
        return false;
    }

    private static Matrix arrayToMatrix(double[] arr) {
        return new Matrix({
            {arr[0], 0, 0, 0, 0},
            {0, arr[1], 0, 0, 0},
            {0, 0, arr[2], 0, 0},
            {0, 0, 0, arr[3], 0},
            {0, 0, 0, 0, arr[4]}
        });
    }

    private double scrubAngle(double theta) {
        while (theta < -Math.PI) {
            theta = theta + (2 * Math.PI);
        }
        while (theta > Math.PI) {
            theta = theta - (2 * Math.PI);
        }
        return theta;
    }
}
