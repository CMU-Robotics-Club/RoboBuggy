package com.roboclub.robobuggy.ui;

import java.awt.Component;
import java.awt.Container;
import java.awt.GridBagConstraints;
import java.awt.GridLayout;

import javax.swing.text.ComponentView;

import com.roboclub.robobuggy.main.MessageLevel;
import com.roboclub.robobuggy.main.RobobuggyLogicException;

/**
 * This is a helper class for the gui containing commonly used gui related helper function
 * @author Trevor Decker
 *
 */
public class GuiUtil {
	
	//This functions splits the current gui pane into the given number of rows and columns
	public static void subPlot(int rows, int cols, int startRow, int startCol, int stopRow, int stopCol,Component comp,Container pane){
		pane.setLayout(null);


		//report errors if something is wrong
		if(startRow > stopRow){
			new RobobuggyLogicException("gui start row is after end row" , MessageLevel.EXCEPTION); 
		}
		
		if(startCol > stopCol){
			new RobobuggyLogicException("gui start col is after end col" , MessageLevel.EXCEPTION); 
		}
		
		if(startCol >= cols){
			new RobobuggyLogicException("gui start col is larger then cols" , MessageLevel.EXCEPTION); 
		}
		
		if(startRow >= rows){
			new RobobuggyLogicException("gui start row is larger then rows" , MessageLevel.EXCEPTION); 
		}
		
		if(stopCol >= cols){
			new RobobuggyLogicException("gui stop col is larger then cols" , MessageLevel.EXCEPTION); 
		}
		
		if(stopRow >= rows){
			new RobobuggyLogicException("gui stop row is larger then rows" , MessageLevel.EXCEPTION); 
		}
		
		if(startCol < 0){
			new RobobuggyLogicException("gui start col is less then 0" , MessageLevel.EXCEPTION); 
		}
		
		if(startRow < 0){
			new RobobuggyLogicException("gui start row is less then 0" , MessageLevel.EXCEPTION); 
		}
		
		if(stopCol < 0){
			new RobobuggyLogicException("gui stop col is less then 0" , MessageLevel.EXCEPTION); 
		}
		
		if(stopRow < 0){
			new RobobuggyLogicException("gui stop row is less then 0" , MessageLevel.EXCEPTION); 
		}
		
		stopRow = stopRow +1;
		stopCol = stopCol +1;
		double width_inCols = stopCol - startCol;
		double height_inRows = stopRow - startRow;
		double frameWidth = pane.getSize().getWidth();
		double frameHeight = pane.getSize().getHeight();
		System.out.println("frameWidth:"+frameWidth+"frameHeight:"+frameHeight);
		
		
		//System.out.println("frameWidth:"+frameWidth+"frameHeight"+frameHeight);
		double widthPerColumn = frameWidth/cols;
		double heightPerRow = frameHeight/rows;
		double width = width_inCols*widthPerColumn;
		double height = height_inRows*heightPerRow;
		double startX = widthPerColumn*startCol;
		double startY = heightPerRow*startRow;
		//System.out.println("startX:"+startX+"startY"+startY+"width"+width+"height"+height);
		
		comp.setBounds((int)startX, (int)startY, (int)width, (int)height);
		pane.add(comp);
	//	System.out.println(pane.getComponentCount());

	}
	
	
}
