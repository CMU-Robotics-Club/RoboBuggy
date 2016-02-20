package com.roboclub.robobuggy.ui;

import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;

/**
 * {@link RobobuggyGUIContainer} used to display an angular graph
 */
public class AngleGraph extends RobobuggyGUIContainer {

	private static final long serialVersionUID = 4524475337756493384L;

	private JTextField reading;
	private Graph graph;
	
	/**
	 * Construct a new {@link AngleGraph}
	 * @param name name of the graph
	 */
	public AngleGraph(String name) {
		reading = new JTextField("", 10);
		reading.setEditable(false);
		JLabel label = new JLabel("   " + name + ": ");
		graph = new Graph();
		this.addComponent(label, 0, 0, 1, .1);
		this.addComponent(graph,0,.1,1,.8);
		this.addComponent(reading, 0, .9, 1, .1);
	}
	
	/**
	 * Update the {@link AngleGraph} with a new value
	 * @param angle new value to display
	 */
	public void updateGraph(double angle) {
		this.graph.updateGraph(angle);
		this.reading.setText(Double.toString(angle));
	}
	
	/**
	 * Private class used to represent a graph
	 */
	private static class Graph extends JPanel {
		private static final long serialVersionUID = 6015150544448011207L;

		private int graphWidth = 200;  //200 is the default  value for the width
		private int graphHeight = 200; //200 is the default value for the height
		private int endx;
		private int endy;

		private int offset = -1;
		private int startX = -1;
		private int startY = -1;
		private double radius = -1; 
		
		/**
		 * Construct a new {@link Graph}
		 */
		public Graph() {
			this.setPreferredSize(new Dimension(graphWidth + 2*offset, 
					graphHeight + 2*offset));	
		}
		
		
		private void updateDimensions(){
			this.graphWidth = (int) (this.getWidth()*.9); //TODO: get ride of magic value
			this.graphHeight = (int) (this.getHeight()*.9);//TODO: get ride of magic value
			this.startX = graphWidth/2 + offset;
			this.startY = graphHeight/2 + offset;
			this.endx = startX;
			this.endy = startY;
			this.offset = 10;//TODO: get ride of magic value
			this.radius = (double)graphWidth/2;
		}
		
		@Override
		public void paintComponent(Graphics g) {
			updateDimensions();
			
			g.setColor(Color.DARK_GRAY);
			g.fillRect(0, 0, graphWidth + 2*offset, graphHeight + 2*offset);
			
			g.setColor(Color.WHITE);
			g.fillOval(offset, offset, graphWidth, graphHeight);
			
			g.setColor(Color.RED);
			g.drawLine(startX, startY, endx, endy);
		}
		
		public void updateGraph(double angle) {
			endx = startX - (int)(radius * Math.sin(Math.toRadians(angle)));
			endy = startY - (int)(radius * Math.cos(Math.toRadians(angle)));
			Gui.getInstance().fixPaint();
		}
	}
}
