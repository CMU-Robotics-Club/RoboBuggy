package com.roboclub.robobuggy.ui;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.File;
import java.util.ArrayList;

import javax.imageio.ImageIO;
import javax.swing.JPanel;

public class VelocityWindow extends RobobuggyGUIContainer{
	public VelocityWindow(){
		GpsPanel gpsPanel = new GpsPanel();
		addComponent(gpsPanel, 0, 0, 1, 1);
	}
}
