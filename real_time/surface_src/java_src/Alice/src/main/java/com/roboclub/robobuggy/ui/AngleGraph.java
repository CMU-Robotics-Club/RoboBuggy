package com.roboclub.robobuggy.ui;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;

import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;

public class AngleGraph extends RoboBuggyGUIContainer {
	private static final long serialVersionUID = 4524475337756493384L;

	private JTextField reading;
	private Graph graph;
	
	public AngleGraph(String name) {
		reading = new JTextField("", 10);
		reading.setEditable(false);
		JLabel label = new JLabel("   " + name + ": ");
		graph = new Graph();
		this.addComponent(label, 0, 0, 1, .1);
		this.addComponent(graph,0,.1,1,.8);
		this.addComponent(reading, 0, .9, 1, .1);
	}
	
	public void updateGraph(int angle) {
		this.graph.updateGraph(angle);
		this.reading.setText(Integer.toString(angle));
	}
	
	private class Graph extends JPanel {
		private static final long serialVersionUID = 6015150544448011207L;

		private int graphWidth = 200;  //200 is the default  value for the width
		private int graphHeight = 200; //200 is the default value for the height
		private int endx;
		private int endy;
		private int offset;
		private int start_x;
		private int start_y;
		private double RADIUS;
		
		public Graph() {
			this.setPreferredSize(new Dimension(graphWidth + 2*offset, 
					graphHeight + 2*offset));	
		}
		
		
		private void updateDimensions(){
			this.graphWidth = (int) (this.getWidth()*.9);
			this.graphHeight = (int) (this.getHeight()*.9);
			this.start_x = graphWidth/2 + offset;
			this.start_y = graphHeight/2 + offset;
			this.endx = start_x;
			this.endy = start_y;
			this.offset = 10;
			this.RADIUS = (double)graphWidth/2;
		}
		
		@Override
		public void paintComponent(Graphics g) {
			updateDimensions();
			
			g.setColor(Color.DARK_GRAY);
			g.fillRect(0, 0, graphWidth + 2*offset, graphHeight + 2*offset);
			
			g.setColor(Color.WHITE);
			g.fillOval(offset, offset, graphWidth, graphHeight);
			
			g.setColor(Color.RED);
			g.drawLine(start_x, start_y, endx, endy);
		}
		
		public void updateGraph(int angle) {
			endx = start_x - (int)(RADIUS * Math.sin(Math.toRadians(angle)));
			endy = start_y - (int)(RADIUS * Math.cos(Math.toRadians(angle)));
			Gui.getInstance().fixPaint();
		}
	}
}
