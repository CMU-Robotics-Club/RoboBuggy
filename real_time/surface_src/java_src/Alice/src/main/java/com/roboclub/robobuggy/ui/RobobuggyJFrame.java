package com.roboclub.robobuggy.ui;

import java.awt.Component;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.GraphicsDevice;
import java.awt.GraphicsEnvironment;
import java.awt.LayoutManager;
import java.awt.event.ComponentEvent;
import java.awt.event.ComponentListener;
import java.io.File;
import java.util.ArrayList;

import javax.imageio.ImageIO;
import javax.swing.*;

/*
 *  This class is for the robobuggy group to have nice control over how Jframe windows look and resize 
 */
public class RobobuggyJFrame extends JFrame  {
	
	/**
	 * 
	 */
	private static final long serialVersionUID = -3566499518806533434L;

	public RobobuggyJFrame(String title,double widthPercentage,double heightPercentage) {
		
		//sets the title based on this frames name
		this.setTitle(title);

		//set the window close default
        this.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);

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

	public void addComponent(Component newComponent, double percentageLeft, double percentageTop, double percentageWidth, double percentageHeight){
		GraphicsDevice gd = GraphicsEnvironment.getLocalGraphicsEnvironment().getDefaultScreenDevice();
		int screenWidth = gd.getDisplayMode().getWidth();
		int screenHeight = gd.getDisplayMode().getHeight();
		newComponent.setBounds((int)(percentageLeft*screenWidth),(int)(percentageTop*screenHeight),(int)(screenWidth*percentageWidth),(int)(screenHeight*percentageHeight));
		if(newComponent instanceof RobobuggyGUIContainer){
			RobobuggyGUIContainer rbGuicontainer = (RobobuggyGUIContainer) newComponent;
			rbGuicontainer.updateSizeing();
		}
		//create a container for keeping track of this components data
		ComponentData thisComponet = new ComponentData(newComponent,
													   percentageLeft,
													   percentageTop,
													   percentageWidth,
													   percentageHeight);
		components.add(thisComponet);
		this.add(newComponent);
		PercentileLayoutManger t = new PercentileLayoutManger(components);
		this.setLayout(t);
	}
	

	
	

}
