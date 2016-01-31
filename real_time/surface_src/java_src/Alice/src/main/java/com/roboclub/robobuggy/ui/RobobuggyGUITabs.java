package com.roboclub.robobuggy.ui;

import javax.swing.JComponent;
import javax.swing.JTabbedPane;

/**
 * 
 * @author Trevor Decker
 * 
 * is a robobuggy gui element that contains many other gui elements in seperate tabs
 *
 */
public class RobobuggyGUITabs extends JTabbedPane {
	
	/**
	 * Adds a tab without an explict title 
	 * @param newTab
	 */
public void addTab(JComponent newTab){
	this.addTab(newTab.getName(), newTab);
}

/**
 *  Adds a tab with an explicit title 
 * @param newTab
 * @param title
 */
public void addTab(JComponent newTab,String title){
	this.addTab(title, newTab);
}

}
