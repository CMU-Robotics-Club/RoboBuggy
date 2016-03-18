package com.roboclub.robobuggy.ui;

/**
 * Creates a window to visualize the velocity results
 * (Really just displays the GPS window on the full screen)
 * @author davidneiman
 *
 */
public class VelocityWindow extends RobobuggyGUIContainer{
	/**
	 * Creates a new VelocityWindow
	 */
	public VelocityWindow(){
		GpsPanel gpsPanel = new GpsPanel();
		addComponent(gpsPanel, 0, 0, 1, 1);
	}
}
