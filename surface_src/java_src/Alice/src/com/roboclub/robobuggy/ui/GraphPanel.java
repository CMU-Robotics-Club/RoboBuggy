package com.roboclub.robobuggy.ui;

import java.awt.Color;
import java.awt.GridLayout;
import javax.swing.BorderFactory;
import javax.swing.JPanel;

public class GraphPanel extends JPanel {
	private static final long serialVersionUID = -5453262887347328140L;

	private AngleGraph steering;
	private AngleGraph roll;
	private AngleGraph pitch;
	private AngleGraph yaw;
	
	public GraphPanel() {
		this.setBorder(BorderFactory.createLineBorder(Color.black));
		this.setLayout(new GridLayout(1,4));
		
		steering = new AngleGraph("STEERING");
		roll = new AngleGraph("ROLL");
		pitch = new AngleGraph("PITCH");
		yaw = new AngleGraph("YAW");
		
		this.add(steering);
		this.add(roll);
		this.add(pitch);
		this.add(yaw);
		
		steering.updateGraph(0.0);
		roll.updateGraph(0.0);
		pitch.updateGraph(0.0);
		yaw.updateGraph(0.0);
	}
}
