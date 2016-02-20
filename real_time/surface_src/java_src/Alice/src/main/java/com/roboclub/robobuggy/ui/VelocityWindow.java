package com.roboclub.robobuggy.ui;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.File;
import java.util.ArrayList;

import javax.imageio.ImageIO;
import javax.swing.JPanel;

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
