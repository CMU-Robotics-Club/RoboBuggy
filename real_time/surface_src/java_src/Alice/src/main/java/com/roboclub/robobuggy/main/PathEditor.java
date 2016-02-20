package com.roboclub.robobuggy.main;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.UnsupportedEncodingException;
import java.util.ArrayList;

import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.nodes.planners.WayPointUtil;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ui.LocTuple;
import com.roboclub.robobuggy.ui.Map;
import com.roboclub.robobuggy.ui.RobobuggyJFrame;

/**
 * Path Editor - Takes a log file and makes a waypoint list from it
 */
public class PathEditor {

	/**
	 * the main method
	 * @param args args
	 */
	public static void main(String[] args) {
		// TODO Auto-generated method stub
			System.out.println("Starting Path Editor");
			try {
				ArrayList<GpsMeasurement> wayPoints =
						WayPointUtil.createWayPointsFromLog("logs/2016-02-13-21-21-41", "sensors_2016-02-13-21-21-41.txt");
				
				
				//save the path to a new jason file type 
				
				//read back in the jason file type
//				ArrayList<Message> waypointsFromFile = WayPointUtil.createWayPointsFromWaypointList("logs/2016-02-13-21-21-41/waypoints.txt");

				//Display the way points 
				RobobuggyJFrame mainWindow = new RobobuggyJFrame("Path Viewer",1.0,1.0);	
				//mainWindow.addComponent(new GpsPanel(), 0.0, 0.0, 1.0, 1.0);
				Map thisMap = new Map();
				mainWindow.addComponent(thisMap,0.0,0.0,1.0,1.0);
				mainWindow.repaint();
				
				//displaying points to the user
				for(int i = 0;i<wayPoints.size();i++){
					System.out.println("lat:"+wayPoints.get(i).getLatitude()+" lon:"+wayPoints.get(i).getLongitude());
					thisMap.addPoint(new LocTuple(wayPoints.get(i).getLatitude(), -wayPoints.get(i).getLongitude()));
					thisMap.repaint();
				}
				
				/*
				WayPointFollowerPlanner planer = new WayPointFollowerPlanner(NodeChannel.UNKNOWN_CHANNEL,wayPoints);
			
				for(int i = 0;i<wayPoints.size();i++){
					double LAT_ERROR = 1/111131.745;
					double LON_ERROR = 1/78846.81;
					for(double latError = -LAT_ERROR;latError<LAT_ERROR;latError+=LAT_ERROR/5){
						for(double lonError = -LON_ERROR;lonError<LON_ERROR;lonError+=LON_ERROR/5){
							planer.updatePositionEstimate(new GPSPoseMessage(new Date(),
								wayPoints.get(i).getLatitude()+latError, wayPoints.get(i).getLongitude()+lonError, 0));
							planer.getCommandedSteeringAngle();
							//TODO plot in a useful way
						}
					}
				}
				*/

				//TODO add zoom and ability to edit 
				
				
				
				
			} catch (UnsupportedEncodingException | FileNotFoundException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (IOException e) {
				e.printStackTrace();
			}
	}

}
