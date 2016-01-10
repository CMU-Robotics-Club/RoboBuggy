package com.roboclub.robobuggy.ui;

import java.awt.Component;
import java.awt.Container;
import java.awt.Graphics;
import java.util.ArrayList;

import javax.swing.JPanel;

public class RoboBuggyGUIContainer extends JPanel{
	ArrayList<ComponentData> components = new ArrayList<ComponentData>();
	String name = "unknown";
	@Override
	public void paint(Graphics g){
		int frameWidth = this.getWidth();
		int frameHeight = this.getHeight();
	//	System.out.println("Component:"+name+"  width:"+frameWidth+"height:"+frameHeight);
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
		add(newComponent);
		this.repaint();
	}
	
}
