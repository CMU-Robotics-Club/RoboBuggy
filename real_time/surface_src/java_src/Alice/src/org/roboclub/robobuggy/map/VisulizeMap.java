package org.roboclub.robobuggy.map;

import java.awt.Container;
import java.awt.FlowLayout;
import java.util.ArrayList;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;

import org.roboclub.robobuggy.coordinateFrame.FrameOfRefrence;
import org.roboclub.robobuggy.coordinateFrame.Pose;
import org.roboclub.robobuggy.main.LogicException;

// TODO make dynamic when user resizes 
// TODO add scale to mapping 
// TODO add moving options
// TODO add save image 


/** 
 * @author Trevor Decker 
 * @version 0.0
 * 
 * library for visulizing the map
 * 
 *  Class for creating a JPanel with a map displayed on it
 */
public class VisulizeMap{
	private JFrame frame = new JFrame();
	private Map thisMap; 
	private FrameOfRefrence pointOfView;
	
	
	//gets the JFrame for the map 
	/*** 
	 * TDOO document 
	 * TODO impment 
	 * @return
	 */
	public JFrame getJFrame(){
		return frame;
	}
	
	/**
	 * TODO document
	 * @param newFrame
	 */
	public void setJFrame(JFrame newFrame){
		this.frame = newFrame;
	}
	
	
	
	
	/**
	 * 
	 * @throws LogicException
	 */
	public void draw() throws LogicException{
		//TODO handle occlusion I think it should be a mask which tells objects where it should not draw

		/*
		MapObject[] objects = thisMap.getClosestNObjects(pointOfView, thisMap.getNumberOfElements());
		for(int i = 0;i<objects.length;i++){
			objects[i].projectTo(pointOfView);
		}
		this.frame.add(comp)
		//TODO actually draw
		 * 
		 */
	}

	
	//TODO add update 
	
	//TODO add gui controls 
	
	
	
	
	//constructor for map visulizer
	/***
	 * TODO document
	 * TODO implment 
	 */
	/*
	public VisulizeMap(Map theMap){
		thisMap = theMap;
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.pack();
		frame.setVisible(true); //TODO move this to an option so that the jframe can be created even if it is not displayed 
		//call updateDisplay which will redraw the map
		updateDisplay();
		drawGUI();
	}
	*/
	
	/***
	 * updates all of the drawing based on the current state of the map 
	 * TODO implment
	 */
	/*
	public void updateDisplay(){
		//TODO consider only updateing the mapobjects that are currently in view 
		//TODO fix frame so that objects don't overlap 
		
		
		Container contentPane = frame.getContentPane();
		contentPane.setLayout(null);
		
		Pose orgin = new Pose(null, null);
		MapObject[] mapObjects = thisMap.getClosestNObjects(orgin,thisMap.getNumberOfElements());
		for (MapObject thisMapObject : mapObjects) {
		//	System.out.println(mapObjects[0].getClass());
		//	Image a = (Image) mapObjects[0];//.paintComponent(frame.getGraphics());
		//	a.paintComponent(frame.getGraphics());
			frame.add(thisMapObject);  //TODO update objects that are already on the frame 
		}
		frame.setSize(500, 500); //TODO make dynamic
	}
	
	public void drawGUI(){
		JButton testButton = new JButton("this is a test");
		int x = frame.getWidth() - 100;
		testButton.setBounds(x, 0, 100, 100);
		frame.add(testButton);
		
		
	}
	*/
	


	
	
}

