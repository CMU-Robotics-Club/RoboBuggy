package com.roboclub.robobuggy.ui;

import Jama.Matrix;
import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Subscriber;
import com.sun.javafx.geom.Vec2d;

import javax.swing.JButton;
import javax.swing.JSlider;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;

/**
 * pose viewer - has a world frame and a buggy relative to it
 */
public class PoseViewer extends RobobuggyGUIContainer{
	private Matrix view;
	private Matrix worldFrame;
	private ArrayList<Matrix> poses;
	private JButton zoomIn, zoomOut;
	private JSlider zoomMag;

	/**
	 * makes a new poseviewer
	 */
	//constructor
	public PoseViewer(){

		zoomIn = new JButton("+");
		zoomIn.setBounds(0, 0, 50, 50);
		zoomIn.setVisible(true);
		zoomIn.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				worldFrame = worldFrame.times(1.1);
				repaint();
			}
		});

		zoomOut = new JButton("-");
		zoomOut.setBounds(0, zoomIn.getY() + zoomIn.getHeight(), 50, 50);
		zoomOut.setVisible(true);
		zoomOut.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				worldFrame = worldFrame.times(0.9);
				repaint();
			}
		});

		this.addComponent(zoomIn, 0, 0, 0.1, 0.1);
		this.addComponent(zoomOut, 0, 0.1, 0.1, 0.1);

		poses = new ArrayList<Matrix>();
		double [][] viewArray = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
		view = new Matrix(viewArray);
		double [][] worldFrameArray = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};

	
		new Subscriber(NodeChannel.POSE.getMsgPath(), new MessageListener() {
			
			@Override
			public void actionPerformed(String topicName, Message m) {
				GPSPoseMessage poseM = (GPSPoseMessage)m;
				double x = poseM.getLatitude();
				double y = poseM.getLongitude();
				double th = Math.PI*poseM.getHeading()/180;
				double [][] anArray = {{Math.cos(th),-Math.sin(th),0,x},{Math.sin(th),Math.cos(th),0,y},{0,0,1,0},{0,0,0,1}};
				Matrix aPose = new Matrix(anArray);
				if(!poses.isEmpty()){
				poses.remove(0);
				}
				poses.add(aPose);
				repaint();
				// TODO Auto-generated method stub
				
			}
		});
		
		
		worldFrame = new Matrix(worldFrameArray);
		//TODO add poses 
	}

	/**
	 * @param m matrix to project
	 * @return the vec that is getting projected
	 */
	public Vec2d projectToView(Matrix m){
		//TODO 
		return new Vec2d(1*m.get(0, 0)+200, -1*m.get(1, 0)+400);
	}

	/**
	 * @param g the graphics instance to draw with
	 * @param m matrix to draw
	 * @param name name of the matrix
	 */
	public void drawMatrix(Graphics g,Matrix m,String name){
		double axisLength = 100;
		double [][] origin = {{0},{0},{0},{1}};
		double [][] xAxis = {{axisLength},{0},{0},{1}};
		double [][] yAxis = {{0},{axisLength},{0},{1}};
		double [][] zAxis = {{0},{0},{axisLength},{1}};
		
		Matrix originMatrix = new Matrix(origin);
		Matrix xMatrix = new Matrix(xAxis);
		Matrix yMatrix = new Matrix(yAxis);
		Matrix zMatrix = new Matrix(zAxis);
		
		Matrix originPoint = m.times(originMatrix);
		Matrix xPoint = m.times(xMatrix);
		Matrix yPoint = m.times(yMatrix);
		Matrix zPoint = m.times(zMatrix);
		
		Vec2d originProjection = projectToView(originPoint);
		Vec2d xProjection = projectToView(xPoint);
		Vec2d yProjection = projectToView(yPoint);
		Vec2d zProjection = projectToView(zPoint);

		//TODO expand to 3d
		
	    g.setColor(Color.RED);
	    g.drawLine((int)originProjection.x, (int)originProjection.y, (int)xProjection.x, (int)xProjection.y);
	    g.setColor(Color.GREEN);
	    g.drawLine((int)originProjection.x, (int)originProjection.y, (int)yProjection.x, (int)yProjection.y);
	    g.setColor(Color.BLUE);
	    g.drawLine((int)originProjection.x, (int)originProjection.y, (int)zProjection.x, (int)zProjection.y);
	    g.setColor(Color.BLACK);
	    g.drawString(name, (int)originProjection.x, (int)originProjection.y);
	}
	
	double get2dth(Matrix m){
		return 180*Math.atan2(m.get(1, 0),m.get(0, 0))/Math.PI;
		
	}
	
	@Override
	public void paintComponent(Graphics g) {
	    super.paintComponent(g);
	    // do your painting here...
	    
	    //update the text portion 
	    g.setColor(Color.BLACK);
	    for(int i = 0;i< poses.size();i++){
	    	Matrix thisPose = poses.get(i);
		    g.drawString("pose:"+i+"\t x:"+thisPose.get(0, 3)+"\t y:"+thisPose.get(1, 3) +"\t th:"+get2dth(thisPose), 50, 25+10*i);

	    }
	    
	    
	    //draws axis
	    drawMatrix( g,worldFrame,"World Frame");
	    
	    for(int i = 0;i<poses.size();i++){
		    drawMatrix( g,poses.get(i),"poses:"+i);
	    }

	}
	

}