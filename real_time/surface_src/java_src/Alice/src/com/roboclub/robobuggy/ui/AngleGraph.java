package com.roboclub.robobuggy.ui;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;

import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;

public class AngleGraph extends JPanel {
	private static final long serialVersionUID = 4524475337756493384L;

	private JTextField reading;
	private Graph graph;
	
	public AngleGraph(String name) {
		this.setLayout(new GridBagLayout());
		this.setBackground(Color.DARK_GRAY);
		
		GridBagConstraints gbc = new GridBagConstraints();
		gbc.gridx = 0;
		gbc.gridy = 0;
		gbc.weighty = 1;
		gbc.weightx = 1;
		gbc.fill = GridBagConstraints.BOTH;
		gbc.anchor = GridBagConstraints.CENTER; 
		
		graph = new Graph();
		this.add(graph);
		
		JPanel panel = new JPanel();
		JLabel label = new JLabel("   " + name + ": ");
		panel.add(label, gbc);
		
		reading = new JTextField("", 10);
		reading.setEditable(false);
		panel.add(reading);
		
		gbc.gridy = 1;
		this.add(panel, gbc);
	}
	
	public void updateGraph(int angle) {
		this.graph.updateGraph(angle);
		this.reading.setText(Integer.toString(angle));
	}
	
	private class Graph extends JPanel {
		private static final long serialVersionUID = 6015150544448011207L;

		private static final int WIDTH = 200;
		private static final int HEIGHT = 200;
		private int endx;
		private int endy;
		private static final int OFFSET = 10;
		private static final int startx = WIDTH/2 + OFFSET;
		private static final int starty = HEIGHT/2 + OFFSET;
		private static final double RADIUS = (double)WIDTH/2;
		
		public Graph() {
			this.setPreferredSize(new Dimension(WIDTH + 2*OFFSET, 
					HEIGHT + 2*OFFSET));
			
			this.endx = startx;
			this.endy = starty;
		}
		
		@Override
		public void paintComponent(Graphics g) {
			g.setColor(Color.DARK_GRAY);
			g.fillRect(0, 0, WIDTH + 2*OFFSET, HEIGHT + 2*OFFSET);
			
			g.setColor(Color.WHITE);
			g.fillOval(OFFSET, OFFSET, WIDTH, HEIGHT);
			
			g.setColor(Color.RED);
			g.drawLine(startx, starty, endx, endy);
		}
		
		public void updateGraph(int angle) {
			endx = startx - (int)(RADIUS * Math.sin(Math.toRadians(angle)));
			endy = starty - (int)(RADIUS * Math.cos(Math.toRadians(angle)));
			
			this.repaint();
		}
	}
}
