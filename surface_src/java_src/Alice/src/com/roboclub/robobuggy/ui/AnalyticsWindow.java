package com.roboclub.robobuggy.ui;

import java.awt.BorderLayout;
import java.awt.Color;
import java.io.File;

import javax.imageio.ImageIO;
import javax.swing.BorderFactory;
import javax.swing.JFrame;
import javax.swing.JPanel;

//What shows up when display graphics is clicked 
public class AnalyticsWindow extends JFrame{
	private static final long serialVersionUID = 1903486493950389240L;
	
	public AnalyticsWindow() {
		
	}
	
	private void setHidden() {
		Gui.updateGraphToggle(false);
	}
}
