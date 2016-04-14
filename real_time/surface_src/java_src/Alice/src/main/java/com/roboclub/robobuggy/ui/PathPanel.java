package com.roboclub.robobuggy.ui;

import com.roboclub.robobuggy.main.RobobuggyConfigFile;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.nodes.localizers.LocTuple;
import com.roboclub.robobuggy.nodes.planners.WayPointUtil;

import javax.swing.JPanel;

import java.awt.Color;
import java.awt.Graphics;
import java.io.IOException;
import java.util.ArrayList;

/**
 * A gui element that shows the waypoints for the path that we are currently trying drive 
 * @author Trevor Decker
 *
 */
public class PathPanel extends JPanel {

	private Map map;

	/**
	 * makes a new PathPanel
	 */
	public PathPanel() {

		map = new Map();
		map.setBounds(0, 0, getWidth(), getHeight());
		add(map);

		try {
			ArrayList<GpsMeasurement> waypoints = WayPointUtil.createWayPointsFromWaypointList(RobobuggyConfigFile.getWaypointSourceLogFile());
			for(int i = 0;i<waypoints.size();i++){
				GpsMeasurement thisPoint = waypoints.get(i);
				map.addPointsToMapTree(Color.BLACK, new LocTuple(thisPoint.getLatitude(), thisPoint.getLongitude()));
				map.repaint();
			}

		} catch (IOException e) {

			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	@Override
	public void paintComponent(Graphics g) {
		super.paintComponent(g);
		map.setBounds(0, 0, getWidth(), getHeight());
	}

}
