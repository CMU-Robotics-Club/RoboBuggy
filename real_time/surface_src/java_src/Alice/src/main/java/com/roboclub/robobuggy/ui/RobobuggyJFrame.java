package com.roboclub.robobuggy.ui;

import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;

import javax.imageio.ImageIO;
import javax.swing.JFrame;
import javax.swing.WindowConstants;
import java.awt.Component;
import java.awt.GraphicsDevice;
import java.awt.GraphicsEnvironment;
import java.io.File;
import java.util.ArrayList;

/**
 *  This class is for the robobuggy group to have nice control over how Jframe windows look and resize 
 */
public class RobobuggyJFrame extends JFrame {
	
	private static final long serialVersionUID = -3566499518806533434L;
	private ArrayList<ComponentData> components = new ArrayList<>();

	/**
	 * Instantiates a RobobuggyJFrame
	 * @param title title of the window
	 * @param widthPercentage percentage of the screen for the width
	 * @param heightPercentage percentage of the screen for the height
	 */
	public RobobuggyJFrame(String title,double widthPercentage,double heightPercentage) {
		
		//sets the title based on this frames name
		this.setTitle(title);

		//set the window close default
        this.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);

		//adds the roboclub icon to the top of the window
		try {
			this.setIconImage(ImageIO.read(new File("images/rc_logo.png")));
		} catch (Exception e) {
			new RobobuggyLogicNotification("Unable to read icon image!", RobobuggyMessageLevel.WARNING);
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


	/**
	 * Adds a new component to the JFrame
	 * @param newComponent the component to add
	 * @param percentageLeft the percentage from the left edge
	 * @param percentageTop the percentage from the top
	 * @param percentageWidth the percentage for the width
	 * @param percentageHeight the percentage for the height
	 */
	public void addComponent(Component newComponent, double percentageLeft, double percentageTop, double percentageWidth, double percentageHeight){
		GraphicsDevice gd = GraphicsEnvironment.getLocalGraphicsEnvironment().getDefaultScreenDevice();
		int screenWidth = gd.getDisplayMode().getWidth();
		int screenHeight = gd.getDisplayMode().getHeight();
		newComponent.setBounds((int)(percentageLeft*screenWidth),
								(int)(percentageTop*screenHeight),
								(int)(screenWidth*percentageWidth),
								(int)(screenHeight*percentageHeight)
		);
		if(newComponent instanceof RobobuggyGUIContainer){
			RobobuggyGUIContainer rbGuicontainer = (RobobuggyGUIContainer) newComponent;
			rbGuicontainer.updateSizing();
		}
		//create a container for keeping track of this components data
		ComponentData thisComponent = new ComponentData(newComponent,
													   percentageLeft,
													   percentageTop,
													   percentageWidth,
													   percentageHeight);
		components.add(thisComponent);
		this.add(newComponent);
		PercentileLayoutManger t = new PercentileLayoutManger(components);
		this.setLayout(t);
	}
	

	
	

}
