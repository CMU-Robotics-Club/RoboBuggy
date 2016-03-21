package com.roboclub.robobuggy.nodes.planners;

import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.ros.NodeChannel;

import java.util.ArrayList;
import java.util.Date;

/**
 * Plans a path based on a set of waypoints
 */
public class WayPointFollowerPlanner extends PathPlannerNode{
	private ArrayList<GpsMeasurement> wayPoints;
	private GPSPoseMessage pose; //TODO change this to a reasonable type
	private int bestGuess = 0; //what point we think we are around

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

	//find the closest way point
	//TODO turn into a binary search
	private int getClosestIndex(){
		double min = Double.MAX_VALUE; //note that the brakes will definitely deploy at this

		int closestIndex = -1;
		for(int i = Math.max(bestGuess-50,0);i<Math.min(wayPoints.size(),bestGuess+50);i++){
			double d = getDistance(pose,wayPoints.get(i));
			if(d < min){
				min = d;
				closestIndex = i;
			}
		}
		return closestIndex;
	}

	@Override
	public double getCommandedSteeringAngle() {
		int closestIndex = getClosestIndex();
		if(closestIndex == -1){
			return 17433504; //A dummy value that we can never get
		}
		bestGuess  = closestIndex;

		double delta = 10/100000.0;
		//pick the first point that is at least delta away
		//pick the point to follow
		int targetIndex = closestIndex;
		while(getDistance(pose,wayPoints.get(targetIndex)) < delta){
			targetIndex = targetIndex+1;
		}



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

		int closestIndex = getClosestIndex();
		if(closestIndex == -1){
			return true;
		}

		// if closest point is too far away throw breaks
		if(getDistance(pose,wayPoints.get(closestIndex)) < 1.0){
			//if we are within 1 meter of any point then do not throw breaks
			return false;
		}
		return true;
	}

	private double getDistance(GPSPoseMessage a, GpsMeasurement b){
		double dx = a.getLatitude() - b.getLatitude();
		double dy = a.getLongitude() - b.getLongitude();
		return Math.sqrt(dx*dx + dy*dy);
	}

}
