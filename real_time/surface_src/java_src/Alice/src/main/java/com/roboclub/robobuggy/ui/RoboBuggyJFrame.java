package com.roboclub.robobuggy.ui;

import java.awt.Component;
import java.awt.Graphics;
import java.awt.GraphicsDevice;
import java.awt.GraphicsEnvironment;
import java.io.File;
import java.util.ArrayList;

import javax.imageio.ImageIO;
import javax.swing.JFrame;

/*
 *  This class is for the robobuggy group to have nice control over how Jframe windows look and resize 
 */
public class RoboBuggyJFrame extends JFrame  {
	
	public RoboBuggyJFrame(String title,double widthPercentage,double heightPercentage) {
		
		//sets the title based on this frames name
		this.setTitle(title);
		
		//adds the roboclub icon to the top of the window
		try {
			this.setIconImage(ImageIO.read(new File("images/rc_logo.png")));
		} catch (Exception e) {
			System.out.println("Unable to read icon image!");
		}
		
		//gets the screen size
		GraphicsDevice gd = GraphicsEnvironment.getLocalGraphicsEnvironment().getDefaultScreenDevice();
		int screenWidth = gd.getDisplayMode().getWidth();
		int screenHeight = gd.getDisplayMode().getHeight();
		double width = widthPercentage*screenWidth;
		double height = heightPercentage*screenHeight;
	
		//makes the window visible 
		this.setBounds(0, 0, (int)width, (int)height);
		this.setVisible(true);
	}
	
ArrayList<ComponentData> components = new ArrayList<ComponentData>();

	
	@Override
	public void paint(Graphics g){
		int frameWidth = this.getWidth();
		int frameHeight = this.getHeight();
		for(int i = 0;i<components.size();i++){
			ComponentData thisComponet = components.get(i);
			thisComponet.component.setBounds((int)(thisComponet.percentageLeft*frameWidth), (int)(thisComponet.percentageTop*frameHeight), (int)(thisComponet.percentageWidth*frameWidth),(int)(thisComponet.percentageHeight*frameHeight));
		}

		super.paint(g);
	}
	
	public void addComponet(Component newComponent,double percentageLeft,double percentageTop,double percentageWidth,double percentageHeight){
		//create a container for keeping track of this components data
		ComponentData thisComponet = new ComponentData();
		thisComponet.component = newComponent;
		thisComponet.percentageLeft = percentageLeft;
		thisComponet.percentageTop = percentageTop;
		thisComponet.percentageWidth = percentageWidth;
		thisComponet.percentageHeight = percentageHeight;
		components.add(thisComponet);
		this.add(newComponent);
	}
	

	
	

}
