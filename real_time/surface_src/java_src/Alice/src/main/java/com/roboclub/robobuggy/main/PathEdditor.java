package com.roboclub.robobuggy.main;

import java.io.FileNotFoundException;
import java.io.UnsupportedEncodingException;
import java.util.ArrayList;
import java.util.Date;

import javax.swing.JButton;

import com.google.gson.Gson;
import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.nodes.planners.WayPointFollowerPlanner;
import com.roboclub.robobuggy.nodes.planners.WayPointUtil;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ui.GpsPanel;
import com.roboclub.robobuggy.ui.RobobuggyJFrame;

public class PathEdditor {

	public static void main(String[] args) {
		// TODO Auto-generated method stub
			System.out.println("Hello World!");
			try {
				ArrayList<GpsMeasurement> wayPoints = WayPointUtil.CreateWayPointsFromLog("logs/2016-02-13-21-21-41/sensors_2016-02-13-21-21-41.txt");
				
				
				//save the path to a new jason file type 
				
				//read back in the jason file type 
				//Display the way points 
				RobobuggyJFrame mainWindow = new RobobuggyJFrame("Path Viewer",1.0,1.0);	
				mainWindow.addComponent(new GpsPanel(), 0.0, 0.0, 1.0, 1.0);
				mainWindow.repaint();
				
				
				Publisher gpsPub = new Publisher(NodeChannel.GPS.getMsgPath());
				for(int i = 0;i<wayPoints.size();i++){
					gpsPub.publish(wayPoints.get(i));
				}
				
				WayPointFollowerPlanner planer = new WayPointFollowerPlanner(NodeChannel.UNKNOWN_CHANNEL,wayPoints);
			
				for(int i = 0;i<wayPoints.size();i++){
					double LAT_ERROR = 1/111131.745;
					double LON_ERROR = 1/78846.81;
					for(double latError = -LAT_ERROR;latError<LAT_ERROR;latError+=LAT_ERROR/5){
						for(double lonError = -LON_ERROR;lonError<LON_ERROR;lonError+=LON_ERROR/5){
							planer.updatePositionEstimate(new GPSPoseMessage(new Date(), wayPoints.get(i).getLatitude()+latError, wayPoints.get(i).getLongitude()+lonError, 0));
							planer.getCommandedSteeringAngle();
							//TODO plot in a useful way
						}
					}
				}

				//TODO add zoom and ability to edit 
				
				
				
				
			} catch (UnsupportedEncodingException | FileNotFoundException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
	}

}
