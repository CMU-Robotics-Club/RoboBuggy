package com.roboclub.robobuggy.ui;

import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;

import java.awt.Component;
import java.awt.Container;

/**
 * This is a helper class for the gui containing commonly used gui related helper function
 * @author Trevor Decker
 *
 */
public class GuiUtil {
	
	/**
	 * This functions splits the current {@link Gui} pane into the given
	 *  number of rows and columns
	 * @param rows number of rows
	 * @param cols number of columns
	 * @param startRow the row to start populating
	 * @param startCol the column to start populating
	 * @param stopRow the row to stop populating at
	 * @param stopCol the column to stop populating at
	 * @param comp the component you are making a subplot of
	 * @param pane the container containing the objects to draw
	 */
	public static void subPlot(int rows, int cols, int startRow, int startCol,
			int stopRow, int stopCol,Component comp,Container pane){
		pane.setLayout(null);


		//report errors if something is wrong
		if(startRow > stopRow){
			new RobobuggyLogicNotification("gui start row is after end row" , RobobuggyMessageLevel.EXCEPTION);
		}
		
		if(startCol > stopCol){
			new RobobuggyLogicNotification("gui start col is after end col" , RobobuggyMessageLevel.EXCEPTION);
		}
		
		if(startCol >= cols){
			new RobobuggyLogicNotification("gui start col is larger then cols" , RobobuggyMessageLevel.EXCEPTION);
		}
		
		if(startRow >= rows){
			new RobobuggyLogicNotification("gui start row is larger then rows" , RobobuggyMessageLevel.EXCEPTION);
		}
		
		if(stopCol >= cols){
			new RobobuggyLogicNotification("gui stop col is larger then cols" , RobobuggyMessageLevel.EXCEPTION);
		}
		
		if(stopRow >= rows){
			new RobobuggyLogicNotification("gui stop row is larger then rows" , RobobuggyMessageLevel.EXCEPTION);
		}
		
		if(startCol < 0){
			new RobobuggyLogicNotification("gui start col is less then 0" , RobobuggyMessageLevel.EXCEPTION);
		}
		
		if(startRow < 0){
			new RobobuggyLogicNotification("gui start row is less then 0" , RobobuggyMessageLevel.EXCEPTION);
		}
		
		if(stopCol < 0){
			new RobobuggyLogicNotification("gui stop col is less then 0" , RobobuggyMessageLevel.EXCEPTION);
		}
		
		if(stopRow < 0){
			new RobobuggyLogicNotification("gui stop row is less then 0" , RobobuggyMessageLevel.EXCEPTION);
		}
		
		stopRow = stopRow +1;
		stopCol = stopCol +1;
		double widthInCols = stopCol - startCol;
		double heightInRows = stopRow - startRow;
		double frameWidth = pane.getSize().getWidth();
		double frameHeight = pane.getSize().getHeight();
		System.out.println("frameWidth:"+frameWidth+"frameHeight:"+frameHeight);
		
		
		//System.out.println("frameWidth:"+frameWidth+"frameHeight"+frameHeight);
		double widthPerColumn = frameWidth/cols;
		double heightPerRow = frameHeight/rows;
		double width = widthInCols*widthPerColumn;
		double height = heightInRows*heightPerRow;
		double startX = widthPerColumn*startCol;
		double startY = heightPerRow*startRow;
		//System.out.println("startX:"+startX+"startY"+startY+"width"+width+"height"+height);
		
		comp.setBounds((int)startX, (int)startY, (int)width, (int)height);
		pane.add(comp);
	//	System.out.println(pane.getComponentCount());
	}
	
	
	
	
}
