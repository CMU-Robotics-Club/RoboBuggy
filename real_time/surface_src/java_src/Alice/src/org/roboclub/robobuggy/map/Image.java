package org.roboclub.robobuggy.map;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

import org.roboclub.robobuggy.coordinateFrame.FrameOfRefrence;
import org.roboclub.robobuggy.coordinateFrame.Pose;

//TODO make a part of polygon 

/**
 * TODO document
 * TODO 
 * 
 * @author Trevor Decker
 * @version 0.0
 *
 */
public class Image extends MapObject{
	BufferedImage img = null;

	/***
	 * TODO document
	 * TODO implement
	 */
	public Image(){
		 refrenceFrame = Pose.zero();
		 //need to have a nonzero setBounds so that the system trys to draw something 
		 setBounds(1,1,100,100);
			File imgFile = new File("/Users/trevordecker/Desktop/test.jpg");
			try{ 
				img = ImageIO.read(imgFile);
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		//TODO
	}
	
	@Override
	/***
	 * TODO document
	 * TODO implement
	 */
	boolean Equals(Object obj) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	/***
	 * TODO document
	 * TODO implement
	 */
	boolean isGreater(Object obj) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	/***
	 * TODO document 
	 * TODO implement
	 */
	boolean isLess(Object obj) {
		// TODO Auto-generated method stub
		return false;
	}
	
	/***
	 * TODO document 
	 * TODO implement 
	 * @param aPoint
	 * @return
	 */
	boolean isInside(Point aPoint){
		//TODO 
		return true;
	}
	
	@Override
	/**
	 * drawing function for the image, will draw the object on the
	 *  jcomponent that the image is added to
	 *  TODO implement 
	 */
	public void paintComponent(Graphics g){
		super.paintComponent(g);
	    g.drawImage(img, 0, 0, this);
	    //TODO add refrence frame stuff for the image
	}


}
