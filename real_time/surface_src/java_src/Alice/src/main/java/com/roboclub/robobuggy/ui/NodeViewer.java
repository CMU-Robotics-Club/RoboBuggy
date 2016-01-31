package com.roboclub.robobuggy.ui;

import java.util.List;
import java.util.Random;

import javax.swing.JButton;

import com.roboclub.robobuggy.main.Robot;
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.NodeChannel;

/**
 * This class creates a gui element for showing the status of every node that is currently running on buggy
 * @author trevordecker
 *
 * Note that currently the list is not updated and it only shows the connections at startup
 */
public class NodeViewer extends RobobuggyGUIContainer{
	
	NodeViewer(){
		Random ran = new Random();
		Robot aRobot = Robot.getInstance();
		List<Node> nodeList = aRobot.getNodeList();
		for(int i = 0;i<nodeList.size();i++){
			JButton thisNode = new JButton(nodeList.get(i).getName());
			this.addComponent(thisNode, ran.nextDouble()*.9, ran.nextDouble()*.9, .1, .1);
		
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
