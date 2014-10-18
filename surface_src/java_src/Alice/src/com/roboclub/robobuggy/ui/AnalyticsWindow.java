package com.roboclub.robobuggy.ui;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Container;
import java.io.File;

import javax.imageio.ImageIO;
import javax.swing.BorderFactory;
import javax.swing.BoxLayout;
import javax.swing.JFrame;
import javax.swing.JPanel;

import com.roboclub.robobuggy.main.Robot;
/**
 * 
 * @author Trevor Decker
 * @author Kevin Brennan 
 *
 * @version 0.5
 * 
 * CHANGELOG: NONE
 * 
 * DESCRIPTION: TODO
 */

//What shows up when display graphics is clicked 
public class AnalyticsWindow extends JFrame{
	private static final long serialVersionUID = 1903486493950389240L;
	
	public AnalyticsWindow() {
		ArduinoPanel arduino_panel = new ArduinoPanel();
		ControlPanel control_panel = new ControlPanel();
		GpsPanel gps_panel = new GpsPanel();
		ImuPanel imu_panel = new ImuPanel();
		
		Container pane = this.getContentPane();
		pane.setLayout(new BoxLayout(pane, BoxLayout.Y_AXIS));
		
		pane.add(arduino_panel);
		pane.add(control_panel);
		pane.add(gps_panel);
		pane.add(imu_panel);
		this.pack();

		// Close reopen main GUI on close
		this.addWindowListener(new java.awt.event.WindowAdapter() {
		    @Override
		    public void windowClosing(java.awt.event.WindowEvent windowEvent) {
		    	Gui.getInstance().mainGuiPane.setVisible(true);
		    	Gui.getInstance().updateGraphToggle(false);
		    }
		});
		
		
		}
	
	
	private void setHidden() {
		Gui.updateGraphToggle(false);
	}
}
