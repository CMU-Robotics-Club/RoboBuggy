package com.roboclub.robobuggy.ui;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;

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
		this.addComponet(label, 0, 0, 1, .1);
		this.addComponet(graph,0,.1,1,.8);
		this.addComponet(reading, 0, .9, 1, .1);
	}
	
	public void updateGraph(int angle) {
		this.graph.updateGraph(angle);
		this.reading.setText(Integer.toString(angle));
	}
	
	private class Graph extends JPanel {
		private static final long serialVersionUID = 6015150544448011207L;

		private int WIDTH = 200;
		private int HEIGHT = 200;
		private int endx;
		private int endy;
		private int OFFSET;
		private int startX;
		private int startY;
		private double RADIUS;
		
		public Graph() {
			this.setPreferredSize(new Dimension(WIDTH + 2*OFFSET, 
					HEIGHT + 2*OFFSET));	
		}
		
		
		private void updateDimensions(){
			this.WIDTH = (int) (this.getWidth()*.9);
			this.HEIGHT = (int) (this.getHeight()*.9);
			this.startX = WIDTH/2 + OFFSET;
			this.startY = HEIGHT/2 + OFFSET;
			this.endx = startX;
			this.endy = startY;
			this.OFFSET = 10;
			this.RADIUS = (double)WIDTH/2;
		}
		
		@Override
		public void paintComponent(Graphics g) {
			updateDimensions();
			
			g.setColor(Color.DARK_GRAY);
			g.fillRect(0, 0, WIDTH + 2*OFFSET, HEIGHT + 2*OFFSET);
			
			g.setColor(Color.WHITE);
			g.fillOval(OFFSET, OFFSET, WIDTH, HEIGHT);
			
			g.setColor(Color.RED);
			g.drawLine(startX, startY, endx, endy);
		}
		
		public void updateGraph(int angle) {
			endx = startX - (int)(RADIUS * Math.sin(Math.toRadians(angle)));
			endy = startY - (int)(RADIUS * Math.cos(Math.toRadians(angle)));
			Gui.getInstance().fixPaint();
		}
	}
}
