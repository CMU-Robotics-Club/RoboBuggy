package com.roboclub.robobuggy.nodes.planners;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.LinkedList;
import java.util.List;

import com.orsoncharts.util.json.JSONArray;
import com.orsoncharts.util.json.JSONObject;
import com.orsoncharts.util.json.parser.JSONParser;
import com.orsoncharts.util.json.parser.ParseException;
import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.messages.PoseMessage;
import com.roboclub.robobuggy.nodes.baseNodes.NodeState;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ui.LocTuple;

/**
 * {@link PathPlannerNode} used to plan a path that follows a previously 
 * recorded path of GPS waypoints, loaded from a log file.
 * 
 * @author Zachary Dawson
 *
 */
public final class GPSTrackPlannerNode extends PathPlannerNode {

	private static final double NEXT_THRESHOLD = 0.1;
	private static final double BRAKE_THRESHOLD = 0.5;
	
	private List<LocTuple> waypoints;
	
	private double commandedAngle;
	private boolean commandedBrake;
	
	/**
	 * Construct a new {@link GPSTrackPlannerNode}
	 * @param channel {@link NodeChannel} on which to broadcast status
	 *  information about the node
	 * @param logFilePath {@link String} representing the path to the log file
	 *  from which the desired GPS path will be extracted.
	 */
	public GPSTrackPlannerNode(NodeChannel channel, String logFilePath) {
		super(channel);
		
		waypoints = new LinkedList<>();
		commandedAngle = 0.0;
		commandedBrake = false;
		
		/* Read in the log file and extract the GPS path */
		System.out.println("Starting to read GPS path from file");
		try {
			JSONParser parser = new JSONParser();
			InputStreamReader reader = new InputStreamReader(new FileInputStream(logFilePath),"UTF-8");	
			JSONObject completeLogFile = (JSONObject) parser.parse(reader); 
			//may take a while, currently working on a solution to read line by line

			JSONArray sensorDataArray = (JSONArray) completeLogFile.get("sensor_data");
			for(Object senObj : sensorDataArray) {
				JSONObject sensor = (JSONObject)senObj;
				String sensorName = (String) sensor.get("name");
				if(sensorName == null){
					new RobobuggyLogicNotification("sensor name is not in this lines log line, this log cannot be repaid",
							RobobuggyMessageLevel.EXCEPTION);
				}else{
					JSONObject sensorParams = (JSONObject) sensor.get("params");
					if(NodeChannel.getNodeForName(sensorName).equals(NodeChannel.GPS)) {
						double latitude = (double) sensorParams.get("latitude");
						double longitude = (double) sensorParams.get("longitude");
						String latDir = (String) sensorParams.get("lat_direction");
						String longDir = (String) sensorParams.get("long_direction");
						boolean north = latDir.equals("N");
						boolean west = longDir.equals("W");
						if(!north)
							latitude *= -1;
						if(west)
							longitude *= -1;
						waypoints.add(new LocTuple(latitude, longitude));
					}
				}
			}
			
		}
		catch(IOException e) {
			new RobobuggyLogicNotification("Error while trying to read from the playback file!", RobobuggyMessageLevel.EXCEPTION);
		} catch (ParseException e) {
			new RobobuggyLogicNotification("Couldn't parse the file given. Make sure it's a proper JSON file", RobobuggyMessageLevel.EXCEPTION);
		}
	}

	/**{@inheritDoc}*/
	@Override
	protected synchronized void updatePositionEstimate(PoseMessage m) {
		LocTuple loc = new LocTuple(m.getLatitude(), m.getLongitude());
		double robotHeading = m.getHeading();
		
		//Find the nearest point in the target path
		int nearestIndex = findNearestIndex(loc);
		int targetIndex = nearestIndex;
		if(targetIndex == -1) {
			new RobobuggyLogicNotification("Could not find nearest point in "
					+ "path loaded from log file",
					RobobuggyMessageLevel.EXCEPTION);
		}
		
		//Determine the next point to aim for (must be a minimum distance away)
		while(nearestIndex < waypoints.size() && 
				loc.getMagDiff(waypoints.get(nearestIndex)) < NEXT_THRESHOLD) {
			nearestIndex++;
		}
		
		//If the next point is too far away, drop the brakes
		if(loc.getMagDiff(waypoints.get(nearestIndex)) < BRAKE_THRESHOLD) {
			commandedBrake = false;
			//Find the desired steering angle
			commandedAngle = getTurnAngle(LocTuple.subtract(waypoints.get(nearestIndex), loc), robotHeading);
		}
		else {
			commandedBrake = true;
			//Set steering to centered if we are braking
			commandedAngle = 0.0;
		}
		
		//Feed the watchdog
		setNodeState(NodeState.ON);
	}
	
	/**
	 * Returns the index of the nearest point in the waypoints list to loc
	 * @param loc {@link LocTuple} to compare to
	 * @return the index of the nearest point in the waypoints list to loc
	 * (-1 if none found)
	 */
	private int findNearestIndex(LocTuple loc) {
		int minIndex = -1;
		double minVal = Double.MAX_VALUE;
		for(int index = 0; index < waypoints.size(); index++) {
			double diff = waypoints.get(index).getMagDiff(loc);
			if(diff < minVal) {
				minVal = diff;
				minIndex = index;
			}
		}
		return minIndex;
	}
	
	/**
	 * Returns the desired turn angle
	 * @param diff {@link LocTuple} representing the difference between the 
	 * target position and the current robot position
	 * @param heading of the robot
	 * @return the desired turn angle
	 */
	private double getTurnAngle(LocTuple diff, double heading) {
		return diff.getHeadingAngle() - heading;
	}

	/**{@inheritDoc}*/
	@Override
	protected synchronized double getCommandedSteeringAngle() {
		return commandedAngle;
	}

	/**{@inheritDoc}*/
	@Override
	protected synchronized boolean getDeployBrakeValue() {
		return commandedBrake;
	}

}
