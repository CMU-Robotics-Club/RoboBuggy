package com.roboclub.robobuggy.main;

import java.awt.*;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.UnsupportedEncodingException;
import java.util.ArrayList;
import java.util.Date;

import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.nodes.planners.WayPointFollowerPlanner;
import com.roboclub.robobuggy.nodes.planners.WayPointUtil;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ui.Gui;
import com.roboclub.robobuggy.ui.LocTuple;

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
						WayPointUtil.createWayPointsFromLog("logs/", "cleaned.txt");
				//save the path to a new jason file type 
				
				//read back in the jason file type
//				ArrayList<Message> waypointsFromFile = WayPointUtil.createWayPointsFromWaypointList("logs/2016-02-13-21-21-41/waypoints.txt");

				//Display the way points 
//				RobobuggyJFrame mainWindow = new RobobuggyJFrame("Path Viewer",1.0,1.0);
				//mainWindow.addComponent(new GpsPanel(), 0.0, 0.0, 1.0, 1.0);
				Gui.getInstance();


				//displaying points to the user
				for(int i = 0;i<wayPoints.size();i++){
					Gui.getInstance().getMainGuiWindow().getAnalyPane().getDataPanel().getGpsPanel().
							addPointsToMapTree(new LocTuple(wayPoints.get(i).getLatitude(),
									-wayPoints.get(i).getLongitude()));
					Gui.getInstance().fixPaint();
				}

				WayPointFollowerPlanner planer = new WayPointFollowerPlanner(NodeChannel.UNKNOWN_CHANNEL,wayPoints);
			
				for(int i = 0;i<wayPoints.size();i++){
					final double latErrorFinal = 1/111131.745;
					final double lonErrorFinal = 1/78846.81;

					Color pointAttemptColor = new Color((float) Math.random(), (float) Math.random(), (float) Math.random());

					for(double latError = -latErrorFinal;latError<latErrorFinal;latError+=latErrorFinal * 4){
						for(double lonError = -lonErrorFinal;lonError<lonErrorFinal;lonError+=lonErrorFinal * 4){
							double lat = wayPoints.get(i).getLatitude() + latError;
							double lon = wayPoints.get(i).getLongitude() + lonError;
							double waypointlat = wayPoints.get(i).getLatitude();
							double waypointlon = wayPoints.get(i).getLongitude();

							double angle = planer.getCommandedSteeringAngle();

							planer.updatePositionEstimate(new GPSPoseMessage(new Date(),
								lat, lon, 0));

							Gui.getInstance().getMainGuiWindow().getAnalyPane().getDataPanel().
									getGpsPanel().addLineToMap(new LocTuple(waypointlat, -waypointlon),
									angle, pointAttemptColor);
							Gui.getInstance().getMainGuiWindow().getAnalyPane().getDataPanel().
									getGpsPanel().addLineToMap(new LocTuple(waypointlat, -waypointlon),
									angle + Math.PI/2, pointAttemptColor.darker());
						}
					}
				}

				//TODO add zoom and ability to edit 
				
				
				
				
			} catch (UnsupportedEncodingException | FileNotFoundException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (IOException e) {
				e.printStackTrace();
			}
	}

}
