package com.roboclub.robobuggy.nodes.planners;

import com.roboclub.robobuggy.messages.DriveControlMessage;
import com.roboclub.robobuggy.messages.PoseMessage;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;

import java.util.Date;

/**
 * Created by vivaanbahl on 1/27/16.
 */
public class SweepNode extends PathPlannerNode {

    private double currentCommandedSteeringAngle = 0;
    private static final double STEERING_ANGLE_LOWER_BOUND = -20;
    private static final double STEERING_ANGLE_UPPER_BOUND = 20;
    private static final double STEERING_ANGLE_INCREMENT = 3;
    private Publisher steeringPublisher;

    /**
     * Construct a new {@link PathPlannerNode}
     *
     * @param channel {@link NodeChannel} on which to broadcast status
     *                information about the node
     */
    public SweepNode(NodeChannel channel)  {
        super(channel);

        steeringPublisher = new Publisher(NodeChannel.DRIVE_CTRL.getMsgPath());
        Thread t1 = new Thread(new Runnable() {
            private boolean sweepUp = false;

            public void run()
            {
                while(true){
                	if(!sweepUp && currentCommandedSteeringAngle <= STEERING_ANGLE_LOWER_BOUND) {
                		sweepUp = true;
                	}else if(sweepUp && currentCommandedSteeringAngle >= STEERING_ANGLE_UPPER_BOUND){
                		sweepUp = false;
                	}
                	
                	if (sweepUp){ 
                		currentCommandedSteeringAngle += STEERING_ANGLE_INCREMENT;
                	}else {
                		currentCommandedSteeringAngle -= STEERING_ANGLE_INCREMENT;
                	}
                	try {
                		Thread.sleep(100);
                	} catch (InterruptedException e) {
                		e.printStackTrace();
                	}	
                	
                	steeringPublisher.publish(new DriveControlMessage(new Date(), currentCommandedSteeringAngle));
            		System.out.println("sending angle: "+currentCommandedSteeringAngle);
                }            }});  
            t1.start();
 
    }

    @Override
    protected void updatePositionEstimate(PoseMessage m) {
        // do nothing here, this is just a simple sweep
        // we don't need to know the position
    }

    @Override
    protected double getCommandedSteeringAngle() {
        return currentCommandedSteeringAngle;
    }

    @Override
    protected boolean getDeployBrakeValue() {
        return false;
    }

    @Override
    public String getName() {
        return "Sweep Test";
    }
}
