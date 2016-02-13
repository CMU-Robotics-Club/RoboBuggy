package com.roboclub.robobuggy.ui;

import java.io.File;

import javax.swing.*;

import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.main.Robot;

public class VelocityPlotter {

	public static void main(String[] args) throws InterruptedException {
		Robot.getInstance();
        //Gui g = Gui.getInstance();
		
		//Code pulled from the Gui constructor
		RobobuggyJFrame mainWindow = new RobobuggyJFrame("MainWindow",1.0,1.0);	
		RobobuggyGUITabs tabs = new RobobuggyGUITabs();
		
		tabs.addTab(new VelocityWindow(), "Velocity"); //Move later, later
		
		tabs.addTab(new MainGuiWindow(),"Home");
		tabs.addTab(new NodeViewer(),"Nodes");

		mainWindow.addComponent(tabs, 0.0, 0.0, 1.0, 1.0);
		while(true){
			mainWindow.repaint();
		}
		
		

		
		/**/
	}
}