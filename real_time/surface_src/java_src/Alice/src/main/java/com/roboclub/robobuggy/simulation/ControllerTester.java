package com.roboclub.robobuggy.simulation;

import Jama.Matrix;
import com.roboclub.robobuggy.messages.DriveControlMessage;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyBaseNode;
import com.roboclub.robobuggy.nodes.baseNodes.PeriodicNode;
import com.roboclub.robobuggy.nodes.localizers.LocTuple;
import com.roboclub.robobuggy.nodes.localizers.LocalizerUtil;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.Subscriber;

/**
 * Created by vivaanbahl on 1/25/17.
 *
 * adapted from @babraham's MATLAB script found in offline/controller
 */
public class ControllerTester extends PeriodicNode {

    // run every 10 ms
    private static final int PERIOD = 1000;
    // wheelbase in meters
    private static final double WHEELBASE = 1.13;
    // assume a velocity of 8 m/s
    private static final double VELOCITY = 8;
    // assume we start from the base of the track
    private static final LocTuple INITIAL_POSITION_LL = new LocTuple(40.441670, -79.9416362);
    // assume we are facing up hill 1
    private static final int INITIAL_HEADING_DEG = 250;

    private long previousTimeMillis;
    private long timeDiffMillis;

    private int heading = INITIAL_HEADING_DEG;
    private LocTuple firstPosition = INITIAL_POSITION_LL;
    private Matrix X;
    private Matrix A;
    private double commandedSteeringAngle = 0;
    private Publisher simulatedPosePub;

    /**
     * Create a new {@link PeriodicNode} decorator
     *
     * @param name
     */
    public ControllerTester(String name) {
        super(new BuggyBaseNode(NodeChannel.AUTO), PERIOD, name);

        double[][] XAsDoubleArr = {
                { LocalizerUtil.deg2UTM(firstPosition).getEasting() },
                { LocalizerUtil.deg2UTM(firstPosition).getNorthing() },
                { VELOCITY },
                { heading },
                { 0 }
        };

        X = new Matrix(XAsDoubleArr);
        previousTimeMillis = System.currentTimeMillis();

        new Subscriber("controller tester", NodeChannel.DRIVE_CTRL.getMsgPath(), ((topicName, m) -> {
            commandedSteeringAngle = ((DriveControlMessage) m).getAngleDouble();
        }));

        simulatedPosePub = new Publisher(NodeChannel.POSE.getMsgPath());
    }

    @Override
    protected void update() {
        timeDiffMillis = System.currentTimeMillis();
        timeDiffMillis -= previousTimeMillis;
        previousTimeMillis = System.currentTimeMillis();

        A = getNewModel(X, heading);


    }

    private Matrix getNewModel(Matrix x, int heading) {
        double[][] matrixAsDoubleArr = {
                { 1, 0, timeDiffMillis * Math.cos(x.get(3, 0)), 0, 0 },
                { 0, 1, timeDiffMillis * Math.sin(x.get(3, 0)), 0, 0 },
                { 0, 0, 1, 0, 0 },
                { 0, 0, 0, 1, timeDiffMillis },
                { 0, 0, Math.tan(heading)/WHEELBASE, 0, 0 }
        };
        return new Matrix(matrixAsDoubleArr);
    }

    @Override
    protected boolean startDecoratorNode() {
        resume();
        return true;
    }

    @Override
    protected boolean shutdownDecoratorNode() {
        return false;
    }
}
