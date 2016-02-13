package com.roboclub.robobuggy.nodes.planners;

import com.roboclub.robobuggy.messages.BrakeControlMessage;
import com.roboclub.robobuggy.messages.DriveControlMessage;
import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;

import java.util.Date;

/**
 * Created by vivaanbahl on 1/27/16.
 */
public class DeadreckoningPlanner extends PathPlannerNode {

    private double currentCommandedSteeringAngle = 0;

    /**
     * Construct a new {@link PathPlannerNode}
     *
     * @param channel {@link NodeChannel} on which to broadcast status
     *                information about the node
     */
    public DeadreckoningPlanner(NodeChannel channel)  {
        super(channel);
 
    }

    @Override
    protected void updatePositionEstimate(GPSPoseMessage m) {
        // do nothing here, this is just a simple sweep
        // we don't need to know the position
    	if(m.getLatitude() < 45.0){
    		//go straight
    		currentCommandedSteeringAngle = 0.0;
    		
    	}else{
    		//turn right 
    		currentCommandedSteeringAngle = 10;
    	}
    	
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
