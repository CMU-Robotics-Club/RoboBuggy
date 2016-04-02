package com.roboclub.robobuggy.ui;

import java.awt.Graphics;
import java.io.IOException;
import java.util.ArrayList;

import javax.swing.JPanel;

import com.roboclub.robobuggy.main.RobobuggyConfigFile;
import com.roboclub.robobuggy.map.ImageMap;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.nodes.planners.WayPointUtil;

/**
 * A gui element that shows the waypoints for the path that we are currently trying drive 
 * @author Trevor Decker
 *
 */
public class PathPanel extends JPanel {
	ImageMap imMap ;
	public PathPanel(){
		imMap = new ImageMap();
		
		try {
			ArrayList<GpsMeasurement> waypoints = WayPointUtil.createWayPointsFromWaypointList("logs/waypoints.txt");
			for(int i = 0;i<waypoints.size();i++){
				GpsMeasurement thisPoint = waypoints.get(i);
				imMap.addPoint(new LocTuple(thisPoint.getLatitude(), thisPoint.getLongitude()));
				System.out.println("point added:"+thisPoint.getLatitude()+","+thisPoint.getLongitude()+" \n");
			}

		} catch (IOException e) {

			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	@Override
	public void paintComponent(Graphics g) {
		super.paintComponent(g);
		imMap.draw(g, getWidth(), getHeight());
	}

}
