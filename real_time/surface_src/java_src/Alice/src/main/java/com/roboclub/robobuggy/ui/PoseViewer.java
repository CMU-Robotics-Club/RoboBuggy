package com.roboclub.robobuggy.ui;

import java.awt.Color;
import java.awt.Graphics;
import java.util.ArrayList;

import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Subscriber;
import com.sun.javafx.geom.Vec2d;

import Jama.Matrix;

public class PoseViewer extends RobobuggyGUIContainer{
	private Matrix view;
	private Matrix worldFrame;
	private ArrayList<Matrix> poses;
	
	//constructor 
	PoseViewer(){
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
				double th = poseM.getHeading();
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
	
	public Vec2d projectToView(Matrix m){
		//TODO 
		return new Vec2d(100*m.get(0, 0)+500, -100*m.get(1, 0)+500);
	}
	
	public void drawMatrix(Graphics g,Matrix m,String name){
		double [][] origin = {{0},{0},{0},{1}};
		double [][] xAxis = {{1},{0},{0},{1}};
		double [][] yAxis = {{0},{1},{0},{1}};
		double [][] zAxis = {{0},{0},{1},{1}};
		
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
		return Math.atan2(m.get(1, 0),m.get(0, 0));
		
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