package com.roboclub.robobuggy.main;

import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.nodes.planners.WayPointFollowerPlanner;
import com.roboclub.robobuggy.nodes.planners.WayPointUtil;
import com.roboclub.robobuggy.ui.LocTuple;
import com.roboclub.robobuggy.ui.Map;

import javax.swing.JFrame;
import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Date;

/**
 * Path Editor - Takes a log file and makes a waypoint list from it
 * @author Trevor Decker
 * @author vivaanbahl
 */
public class PathEditor {

	/**
	 * the main method
	 * @param args args
	 */
	public static void main(String[] args) {
		double currentHeading = 0;
		final double latErrorFinal = 2/111131.745;
		final double lonErrorFinal = 2/78846.81;
		Map m;

			try {
				RobobuggyConfigFile.loadConfigFile();

				JFrame f = new JFrame("path editor");
				f.setBounds(0, 0, 500, 500);
				m = new Map();
				m.setBounds(0, 0, 500, 500);
				m.setVisible(true);
				f.add(m);
				f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
				f.setVisible(true);

				ArrayList<GpsMeasurement> wayPoints =
						WayPointUtil.createWayPointsFromWaypointList(RobobuggyConfigFile.getWaypointSourceLogFile());

//				Gui.getInstance();


				//displaying points to the user
				for(int i = 0;i<wayPoints.size();i++){
							m.addPointsToMapTree(Color.BLUE,new LocTuple(wayPoints.get(i).getLatitude(),
									wayPoints.get(i).getLongitude()));
					m.repaint();
				}

				WayPointFollowerPlanner planer = new WayPointFollowerPlanner(wayPoints);




				for(int i = 1;i<wayPoints.size();i++){
					for(double latError = -latErrorFinal;latError<=latErrorFinal;latError+=5*latErrorFinal){
						for(double lonError = -lonErrorFinal;lonError<=lonErrorFinal;lonError+=5*lonErrorFinal){
							double lat = wayPoints.get(i).getLatitude() + latError;
							double lon = wayPoints.get(i).getLongitude() + lonError;
							planer.updatePositionEstimate(new GPSPoseMessage(new Date(), lat, lon, currentHeading));
							double angle = Math.PI*planer.getCommandedSteeringAngle()/180;

							m.addPointsToMapTree(Color.RED,new LocTuple(lat, lon));
							m.addLineToMap(new LocTuple(lat, lon), angle, Color.RED, false);
							m.addLineToMap(new LocTuple(wayPoints.get(i).getLatitude(), wayPoints.get(i).getLongitude()), currentHeading, Color.YELLOW, false);
							m.repaint();

						}
					}
				}

				//TODO add zoom and ability to edit




			} catch (IOException e) {
				e.printStackTrace();
			}
	}

}
