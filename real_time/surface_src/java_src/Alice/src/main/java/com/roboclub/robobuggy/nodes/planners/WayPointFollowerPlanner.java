package com.roboclub.robobuggy.nodes.planners;

import java.util.ArrayList;
import java.util.Date;

import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.ros.NodeChannel;

/**
 * Plans a path based on a set of waypoints
 */
public class WayPointFollowerPlanner extends PathPlannerNode{
	private ArrayList<GpsMeasurement> wayPoints;
	private GPSPoseMessage pose; //TODO change this to a reasonable type


	/**
	 * @param channel for buggybasenode
	 * @param wayPoints the list of waypoints to follow
	 */
	public WayPointFollowerPlanner(NodeChannel channel,ArrayList wayPoints) {
		super(channel);
		this.wayPoints = wayPoints;
		pose = new GPSPoseMessage(new Date(0), 0, 0, 0);// temp measurment 
	}

	@Override
	public void updatePositionEstimate(GPSPoseMessage m) {
		pose = m;
		
	}

	@Override
	public double getCommandedSteeringAngle() {
		//find the closest way point 
		double min = 10000;//note that the breaks will defiantly deploy at this 
		int closestIndex = -1;
		for(int i = 0;i<wayPoints.size();i++){
			double d = getDistince(pose,wayPoints.get(i));
			if(d < min){
				min = d;
				closestIndex = i;
			}
		}
		
		System.out.println("Closest Point: "+closestIndex);
		
		//pick the point to follow 
		int targetIndex = closestIndex + 10;

		
		//if we are out of points then just go straight
		if(targetIndex >= wayPoints.size())
		{
			return 0;
		}
		
		GpsMeasurement targetPoint = wayPoints.get(targetIndex);
						
		//find a path from our current location to that point 
		double dx = targetPoint.getLatitude() - pose.getLatitude();
		double dy = targetPoint.getLongitude() - pose.getLongitude();
		double desiredHeading = 180*Math.atan2(dy, dx)/Math.PI;
		
		
		//find the angle we need to reach that point 
		//return pose.getHeading() - desiredHeading;
		return  desiredHeading - pose.getHeading();

	}

	@Override
	protected boolean getDeployBrakeValue() {
		// if closest point is too far away throw breaks 
		for(int i = 0;i<wayPoints.size();i++){
			double d = getDistince(pose,wayPoints.get(i));
			if(d < 1.0){
				//if we are within 1 meter of any point then do not throw breaks 
				return false;
			}
		}
		
		return true;
	}
	
	private double getDistince(GPSPoseMessage a,GpsMeasurement b){
		double dx = a.getLatitude() - b.getLatitude();
		double dy = a.getLongitude() - b.getLongitude();
		return Math.sqrt(dx*dx + dy*dy);
	}

}
