package com.roboclub.robobuggy.ui;

import java.awt.Component;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.LayoutManager;
import java.util.ArrayList;


class PercentileLayoutManger implements LayoutManager{
	ArrayList<ComponentData> components;
	
	PercentileLayoutManger(ArrayList<ComponentData> components){
		this.components = components;
	}
	
	@Override
	public void addLayoutComponent(String name, Component comp) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void removeLayoutComponent(Component comp) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public Dimension preferredLayoutSize(Container parent) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Dimension minimumLayoutSize(Container parent) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void layoutContainer(Container parent) {
		int frameWidth = parent.getWidth();
		int frameHeight = parent.getHeight();
		for(int i = 0;i<this.components.size();i++){
			Component thisComponent = this.components.get(i).getComponent();
			int x = (int)(this.components.get(i).getPercentageLeft()*frameWidth);
			int y = (int)(this.components.get(i).getPercentageTop()*frameHeight);
			int width = (int)(this.components.get(i).getPercentageWidth()*frameWidth);
			int height = (int)(this.components.get(i).getPercentageHeight()*frameHeight);
			thisComponent.setBounds(x, y, width, height);
		}		
	}
	
}