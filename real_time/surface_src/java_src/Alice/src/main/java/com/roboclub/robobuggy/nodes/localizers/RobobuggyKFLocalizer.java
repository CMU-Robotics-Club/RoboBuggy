package com.roboclub.robobuggy.nodes.localizers;

import Jama.Matrix;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyBaseNode;
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
    private static final int X_GLOBAL_ROW = 0;
    private static final int Y_GLOBAL_ROW = 1;
    private static final int Y_BODY_ROW = 2;
    private static final int HEADING_GLOBAL_ROW = 3;
    private static final int HEADING_VEL_ROW = 4;

    public static final double WHEELBASE_IN_METERS = 1.13; // meters
    private static final int UTMZONE = 17;
    private final UTMTuple initialLocationGPS; // new LocTuple(40.441670, -79.9416362));
    private static final double INITIAL_HEADING_IN_RADS = 4.36; // rad

    // The state consists of 4 elements:
    //      x        - x position in the world frame, in meters
    //      y        - y position in the world frame, in meters
    //      dy_body  - forward velocity in the body frame, in meters/s
    //      heading  - heading, or yaw angle, in the world frame, in rad
    //      dheading - angular velocity in the world frame, in rad/s
    private Matrix x;                   // current state
    private Matrix R;                   // measurement noise covariance matrix
    private Matrix P;                   // covariance matrix
    private Matrix Q_gps;               // model noise covariance matrix
    private Matrix Q_encoder;

    // output matrices
    private Matrix C_gps;               // a description of how the GPS impacts x
    private Matrix C_encoder;           // how the encoder affects x

    private long lastTime;              // the last time we updated the current position estimate
    private UTMTuple lastGPS;           // the most recent value of the GPS coordinates, expressed as a UTM measurement
    private double lastEncoder;         // deadreckoning value
    private long lastEncoderTime;       // the most recent time of the encoder reading, used for getting buggy velocity
    private double steeringAngle = 0;   // current steering angle of the buggy

    public Matrix getCurrentState() {
        return x;
    }

    /**
     * Create a new {@link PeriodicNode} decorator
     *
     * @param period of the periodically executed portion of the node
     * @param name the name of the node
     * @param initialPosition the initial position of the localizer
     */
    protected RobobuggyKFLocalizer(int period, String name, LocTuple initialPosition) {
        super(new BuggyBaseNode(NodeChannel.POSE), period, name);
        posePub = new Publisher(NodeChannel.POSE.getMsgPath());

        initialLocationGPS = LocalizerUtil.deg2UTM(initialPosition);

        // set initial state
        lastTime = new Date().getTime();
        lastEncoder = 0;
        lastEncoderTime = lastTime;
        lastGPS = initialLocationGPS;

        double[][] x2D = {
                { initialLocationGPS.getEasting() },
                { initialLocationGPS.getNorthing() },
                { 0 },
                { INITIAL_HEADING_IN_RADS },
                { 0 }
        };
        x = new Matrix(x2D);

        double[] rArray = {4, 4, 0.25, 0.01, 0.01};
        double[] pArray = {25, 25, 0.25, 2.46, 2.46};

        R = arrayToMatrix(rArray);
        P = arrayToMatrix(pArray);

        double[][] qGPS2D = {
                {4, 0, 0},
                {0, 4, 0},
                {0, 0, 0.02},
        };
        Q_gps = new Matrix(qGPS2D);

        double[][] qEncoder2D = {{0.25}};
        Q_encoder = new Matrix(qEncoder2D);

        double[][] cGPS2D = {
                {1, 0, 0, 0, 0},
                {0, 1, 0, 0, 0},
                {0, 0, 0, 1, 0},
        };
        C_gps = new Matrix(cGPS2D);

        double[][] cEncoder2D = {
                {0, 0, 1, 0, 0},
        };
        C_encoder = new Matrix(cEncoder2D);

        // add all our subscribers for our current state update stream
        // Every time we get a new sensor update, trigger the new kalman update
        setupGPSSubscriber();
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
            double[][] z2D = {{ bodySpeed }};
            Matrix z = new Matrix(z2D);

            kalmanFilter(C_encoder, Q_encoder, z);
        }));
    }

    private void setupGPSSubscriber() {
        new Subscriber("KF Localizer", NodeChannel.GPS.getMsgPath(), ((topicName, m) -> {
            GpsMeasurement gpsLoc = (GpsMeasurement) m;
            LocTuple loc = new LocTuple(gpsLoc.getLatitude(), gpsLoc.getLongitude());
            UTMTuple gps = LocalizerUtil.deg2UTM(loc);
            double dx = gps.getEasting() - lastGPS.getEasting();
            double dy = gps.getNorthing() - lastGPS.getNorthing();
            lastGPS = gps;

            // don't update angle if we did not move a lot
            double heading = Math.atan2(dy, dx);
            if ((dx * dx + dy * dy) < 0.25) {
                heading = x.get(HEADING_GLOBAL_ROW, 0);
            }
            // close the loop, lock initial angle
            if (Math.abs(gps.getEasting() - initialLocationGPS.getEasting())
                    + Math.abs(gps.getNorthing() - initialLocationGPS.getNorthing()) < 10.0) {
                // todo should this be previous heading?
                heading = INITIAL_HEADING_IN_RADS;
            }

            // measurement
            double[][] z2D = {
                    { gps.getEasting() },
                    { gps.getNorthing() },
                    { heading },
            };

            Matrix z = new Matrix(z2D);

            kalmanFilter(C_gps, Q_gps, z);
        }));
    }

    private void setupWheelSubscriber() {
        new Subscriber("htGpsLoc", NodeChannel.STEERING.getMsgPath(), ((topicName, m) -> {
            SteeringMeasurement steerM = (SteeringMeasurement) m;
            steeringAngle = Math.toRadians(steerM.getAngle());
        }));
    }

    @Override
    protected void update() {
        Matrix x_predict = propagate();

        UTMTuple utm = new UTMTuple(UTMZONE, 'T', x_predict.get(X_GLOBAL_ROW, 0),
                x_predict.get(Y_GLOBAL_ROW, 0));
        LocTuple latLon = LocalizerUtil.utm2Deg(utm);
        posePub.publish(new GPSPoseMessage(new Date(), latLon.getLatitude(),
                latLon.getLongitude(), x_predict.get(HEADING_GLOBAL_ROW, 0), x));
    }

    // Kalman filter step 0: Generate the motion model for the buggy
    private Matrix getMotionModel(double dt) {
        double[][] motionModel2D = {
                // x y dy_b heading dheading
                { 1, 0, dt * Math.cos(x.get(HEADING_GLOBAL_ROW, 0)), 0, 0 }, // x
                { 0, 1, dt * Math.sin(x.get(HEADING_GLOBAL_ROW, 0)), 0, 0 }, // y
                { 0, 0, 1, 0, 0 }, // dy_b
                { 0, 0, 0, 1, dt }, // heading
                { 0, 0, Math.tan(steeringAngle) / WHEELBASE_IN_METERS, 0, 0 }, // dheading
        };

        return new Matrix(motionModel2D);
    }

    /*
    Kalman filter step 1: Use the motion model to predict the next state
    and covariance matrix. (_pre = predicted)
    Kalman filter step 2: Update the prediction based off of measurements /
    sensor readings
    
    <x[k-1], p[k-1]> ---> {A} ---> <xhat[k], phat[k]> ---> |
          A                                               |---> {filter} ---> <x[k], p[k]> ---|
          |                                        z ---> |                                   |
          |                                                                                   |
          ------------------------------------------------------------------------------------|
    */
    private void kalmanFilter(Matrix C, Matrix Q, Matrix z) {
        // update time
        Date now = new Date();
        double dt = (now.getTime() - lastTime) / 1000.0;
        lastTime = now.getTime();

        Matrix A = getMotionModel(dt);

        /*
        the predict step is responsible for determining the estimate of the next state
        What we do is we take our current state (x[k-1]) and current noise estimate
        (p[k-1]), and plug them into the motion model (A). This lets us determine our
        prediction state (xhat[k]) and our prediction noise (phat[k])

        Predict:
            x_pre = A * x
            P_pre = A * P * A' + R
        */
        Matrix x_pre = A.times(x);
        Matrix P_pre = A.times(P).times(A.transpose());
        P_pre = P_pre.plus(R);

        x_pre.set(HEADING_GLOBAL_ROW, 0, clampAngle(x_pre.get(HEADING_GLOBAL_ROW, 0)));
        x_pre.set(HEADING_VEL_ROW, 0, clampAngle(x_pre.get(HEADING_VEL_ROW, 0)));

        /* 
        the update step is responsible for updating the current state, and resolving
        discrepancies between the prediction and actual state
        
        What we do is we take our measurements (z) that were captured from sensors,
        as well as the predictions from the previous step (xhat[k], phat[k]) and run
        them through the filter
        
        This produces your next state (x[k]) and your next noise estimation (p[k]),
        which connect in a feedback loop to become the new current state

        Update:
            r = z - (C * x_pre)
            K = P_pre * C' * inv((C * P_pre * C') + Q) // gain
            x = x_pre + (K * r)
            P = (I - (K * C)) * P_pre
        */
        Matrix residual = z.minus(C.times(x_pre));
        Matrix K = C.times(P_pre);
        K = K.times(C.transpose());
        K = K.plus(Q);
        K = P_pre.times(C.transpose()).times(K.inverse());

        x = x_pre.plus(K.times(residual));
        P = Matrix.identity(5, 5).minus(K.times(C));
        P = P.times(P_pre);

        x.set(HEADING_GLOBAL_ROW, 0, clampAngle(x.get(HEADING_GLOBAL_ROW, 0)));
        x.set(HEADING_VEL_ROW, 0, clampAngle(x.get(HEADING_VEL_ROW, 0)));
    }

    // Propagate the motion model forward in time since the last sensor reading
    // not part of the Kalman algorithm
    private Matrix propagate() {
        Date now = new Date();
        double dt = (now.getTime() - lastTime) / 1000.0;

        // x_pre = A * x
        Matrix A = getMotionModel(dt);
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
        double[][] arr2D = {
                {arr[0], 0, 0, 0, 0},
                {0, arr[1], 0, 0, 0},
                {0, 0, arr[2], 0, 0},
                {0, 0, 0, arr[3], 0},
                {0, 0, 0, 0, arr[4]}
        };
        return new Matrix(arr2D);
    }

    // clamp all the angles between -pi and +pi
    private double clampAngle(double theta) {
        while (theta < -Math.PI) {
            theta = theta + (2 * Math.PI);
        }
        while (theta > Math.PI) {
            theta = theta - (2 * Math.PI);
        }
        return theta;
    }
}
