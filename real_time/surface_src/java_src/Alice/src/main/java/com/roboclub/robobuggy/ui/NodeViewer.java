package com.roboclub.robobuggy.ui;

import com.roboclub.robobuggy.main.Robot;
import com.roboclub.robobuggy.ros.Node;

import javax.swing.JButton;
import java.util.ArrayList;
import java.util.List;

/**
 * This class creates a gui element for showing the status of every node that is currently running on buggy
 * @author trevordecker
 *
 * Note that currently the list is not updated and it only shows the connections at startup
 */
public class NodeViewer extends RobobuggyGUIContainer{

	// use this for organizing them in a list
	private ArrayList<Double> nodeLocList;
	private static final double LEFT_PADDING = .01;
	private static final double HEIGHT_PADDING = .1;

	/**
	 * Constructor for the NodeViewer - gets all the current nodes and adds them to the screen
	 */
	public NodeViewer(){
		nodeLocList = new ArrayList<>();
		nodeLocList.add(-0.1);

		Robot aRobot = Robot.getInstance();
		List<Node> nodeList = aRobot.getNodeList();

		for (Node aNodeList : nodeList) {
			JButton thisNode = new JButton(aNodeList.getName());
			double buttonYpos = nodeLocList.get(nodeLocList.size() - 1) + HEIGHT_PADDING;
			nodeLocList.add(buttonYpos);

			this.addComponent(thisNode, LEFT_PADDING, buttonYpos, .1, .1);

		}
		
		/*
		NodeChannel[] nodes = NodeChannel.values();
		double width = 1.0/nodes.length;
		for(int i = 0;i<nodes.length;i++){
			JButton thisNode = new JButton(nodes[i].getName());
			this.addComponent(thisNode, ran.nextDouble()*.9, ran.nextDouble()*.9, .1, .1);
		}
		*/
	}
	
	
}
