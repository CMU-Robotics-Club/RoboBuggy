package com.roboclub.robobuggy.ui;

import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.map.ImageMap;

import javax.imageio.ImageIO;
import javax.swing.JPanel;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.File;
import java.util.ArrayList;

/**
 * the map, it plots points where we are based on gps
 */
public class Map extends JPanel {
	private ImageMap imMap; 


	/**
	 * makes the new map
	 */
	public Map(){
		imMap = new ImageMap();
	
	}


	/**
	 * @param newPoint the point to add to the map
	 */
	public void addPoint(LocTuple newPoint){
		imMap.addPoint(newPoint);
		this.repaint();
	}

	/**
	 * @param newPoint point to add
	 * @param orientation the orientation to put the arrow at
	 */
	public void addArrow(LocTuple newPoint,double orientation){
		//TODO
	}

	@Override
	public void paintComponent(Graphics g) {
		super.paintComponent(g);
		imMap.draw(g, getWidth(), getHeight());
	}
	
	
}
