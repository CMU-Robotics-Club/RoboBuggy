package com.roboclub.robobuggy.ui;

import java.awt.Component;
import java.awt.Graphics;
import java.util.ArrayList;

import javax.swing.JPanel;

public class RobobuggyGUIContainer extends JPanel{
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

	
	
	
	public void updateSizeing(){
		int frameWidth = this.getWidth();
		int frameHeight = this.getHeight();
		for(int i = 0;i<components.size();i++){
			ComponentData thisComponet = components.get(i);
			//calculate the size of the subcomponent
			int subComponetX = (int)(thisComponet.getPercentageLeft()*frameWidth);
			int subComponetY = (int)(thisComponet.getPercentageTop()*frameHeight);
			int subComponentWidth = (int)(thisComponet.getPercentageWidth()*frameWidth);
			int subComponentHeight = (int)(thisComponet.getPercentageHeight()*frameHeight);
			//actually set the size bound 
			thisComponet.getComponent().setBounds(subComponetX,subComponetY,subComponentWidth,subComponentHeight);
			// we don't know if the component is a robobuggy container or a primitive component, 
			// if it is a primitive then we are finished otherwise we need to recursively apply the size change 
			if(thisComponet.getComponent() instanceof RobobuggyGUIContainer){
				RobobuggyGUIContainer rbGuicontainer = (RobobuggyGUIContainer) thisComponet.getComponent();
				rbGuicontainer.updateSizeing();
			}
		}
		
	}
	
	public void addComponent(Component newComponent, double percentageLeft, double percentageTop, double percentageWidth, double percentageHeight){
		//create a container for keeping track of this components data
		ComponentData thisComponet = new ComponentData(newComponent,
				   percentageLeft,
				   percentageTop,
				   percentageWidth,
				   percentageHeight);
		components.add(thisComponet);
		add(newComponent);
		this.repaint();
		PercentileLayoutManger t = new PercentileLayoutManger(components);
		this.setLayout(t);
	}
	
}
