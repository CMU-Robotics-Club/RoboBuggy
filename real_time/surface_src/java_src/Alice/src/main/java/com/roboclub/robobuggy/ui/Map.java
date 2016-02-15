package com.roboclub.robobuggy.ui;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.File;

import javax.imageio.ImageIO;

public class Map {
	private BufferedImage map;

	Map(){
		try {
			map = ImageIO.read(new File("images/lat_long_course_map.png"));
		} catch(Exception e) {
			System.out.println("Unable to open map!");
		}
	}
	
	
	@Override
	public void paintComponent(Graphics g) {
		setup();
		super.paintComponent(g);
		if (!setup){
			setup();
			setup = true;
		}
		Graphics2D g2d = (Graphics2D) g.create();

		g.drawImage(map, 0, 0, frameWidth, frameHeight, Color.black, null);

		for (int i = 0; i < locs.size(); i++) {
			LocTuple mTuple = locs.get(i);
			drawTuple(g2d, mTuple);
		}
		g2d.dispose();
	}
	
	
	
}
