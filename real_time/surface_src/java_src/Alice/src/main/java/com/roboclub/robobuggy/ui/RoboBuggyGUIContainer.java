package com.roboclub.robobuggy.ui;

import java.awt.Component;
import java.awt.Graphics;
import java.util.ArrayList;

import javax.swing.JPanel;

public class RoboBuggyGUIContainer extends JPanel{
	/**
	 * 
	 */
	private static final long serialVersionUID = -7096009898048816618L;
	/**************************** internal variables **********************************/
	private ArrayList<ComponentData> components = new ArrayList<ComponentData>();
	protected String name = "unknown";
	private int thisComponentWidth = 0;			//how wide in pixels that the component should be drawn as
	private int thisComponentHeight = 0;	 	//how tall in pixels that the component should be drawn as
	
	
	
	/*************************** class public metheds *********************************/

	@Override
	public void paint(Graphics g){
		int frameWidth = this.getWidth();
		int frameHeight = this.getHeight();
	//	System.out.println("Component:"+name+"  width:"+frameWidth+"height:"+frameHeight);
/*
		int frameWidth = this.getWidth();
		int frameHeight = this.getHeight();
	//	System.out.println("Component:"+name+"  width:"+frameWidth+"height:"+frameHeight);
		for(int i = 0;i<components.size();i++){
			ComponentData thisComponet = components.get(i);
			thisComponet.component.setBounds((int)(thisComponet.percentageLeft*frameWidth), (int)(thisComponet.percentageTop*frameHeight), (int)(thisComponet.percentageWidth*frameWidth),(int)(thisComponet.percentageHeight*frameHeight));
		}
		*/
		super.paint(g);
	}
	
	
	
	public void updateSizeing(){
		int frameWidth = this.getWidth();
		int frameHeight = this.getHeight();
	//	System.out.println("frameWidth "+frameWidth+" frameHeight"+frameHeight);
		for(int i = 0;i<components.size();i++){
			ComponentData thisComponet = components.get(i);
			//calculate the size of the subcomponent
			int subComponetX = (int)(thisComponet.percentageLeft*frameWidth);
			int subComponetY = (int)(thisComponet.percentageTop*frameHeight);
			int subComponentWidth = (int)(thisComponet.percentageWidth*frameWidth);
			int subComponentHeight = (int)(thisComponet.percentageHeight*frameHeight);
			//actually set the size bound 
			thisComponet.component.setBounds(subComponetX,subComponetY,subComponentWidth,subComponentHeight);
			// we don't know if the component is a robobuggy container or a primitive component, 
			// if it is a primitive then we are finished otherwise we need to recursively apply the size change 
			if(thisComponet.component instanceof RoboBuggyGUIContainer){
				RoboBuggyGUIContainer rbGuicontainer = (RoboBuggyGUIContainer) thisComponet.component;
				rbGuicontainer.updateSizeing();
			}
		}
		
	}
	
	public void addComponent(Component newComponent, double percentageLeft, double percentageTop, double percentageWidth, double percentageHeight){
		//create a container for keeping track of this components data
		ComponentData thisComponet = new ComponentData();
		thisComponet.component = newComponent;
		thisComponet.percentageLeft = percentageLeft;
		thisComponet.percentageTop = percentageTop;
		thisComponet.percentageWidth = percentageWidth;
		thisComponet.percentageHeight = percentageHeight;
		components.add(thisComponet);
		add(newComponent);
		this.repaint();
		testLayout t = new testLayout(components);
		this.setLayout(t);
	}
	
}
