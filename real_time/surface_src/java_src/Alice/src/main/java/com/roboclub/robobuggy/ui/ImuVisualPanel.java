package com.roboclub.robobuggy.ui;

import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Subscriber;

import javax.imageio.ImageIO;
import javax.swing.JPanel;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.File;
import java.util.Arrays;


/**
 * Creates a panel for visualizing the IMU results
 * @author davidneiman
 *
 */
public class ImuVisualPanel extends JPanel {	
	
	//Variables go here
	private static final long serialVersionUID = 42L; //This probably should be changed
	private double yaw = 0, pitch = 0, roll = 0;
	private double sideLength;
	private boolean setup;
	private int frameWidth;
	private int frameHeight;
	private BufferedImage background;
	//It'll need a subscriber eventually
	
	//Modified from GpsPanel
	@SuppressWarnings("unused") //this subscriber is used to generate callbacks 
	private Subscriber imuSub;
	
	/**
	 * Creates an IMU panel
	 */
	public ImuVisualPanel(){
		try {
			background = ImageIO.read(new File("images/background.png"));
		} catch(Exception e) {
			new RobobuggyLogicNotification("Unable to read background image!", RobobuggyMessageLevel.WARNING);
		}
		setup = false;
		
		
		imuSub = new Subscriber("uiImu", NodeChannel.IMU.getMsgPath(), new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				
				//If I'm reading this correctly, this should activate by itself when I get a message
				
				yaw = Math.PI/180*((ImuMeasurement)m).getYaw();
				pitch = Math.PI/180*((ImuMeasurement)m).getPitch();
				roll = Math.PI/180*((ImuMeasurement)m).getRoll();
				
				//Keep this
			    Gui.getInstance().fixPaint();
			}
		});

	}
	
	private void setup() {
		frameWidth = getWidth();
		frameHeight = getHeight();
		sideLength = 0.4*Math.min(frameWidth, frameHeight);
	}
	
	private void rotateYPR(double[] xyz, double yaw, double pitch, double roll){
		//I assume positive yaw is a right turn
		//positive pitch is upward
		//and positive roll is right side down
		//as per Wikipedia's definition
		
		//I define +x as to the right on screen, +y as down on screen, and +z as out of the screen
		//I assume the buggy is facing right (+x direction)

		//Also, I'm using aliasing intentionally here
		
		//Can I really just keep applying transformations in series like this?
		
		//Do yaw:
		double temp = xyz[0]*Math.cos(yaw) - xyz[2]*Math.sin(yaw);
		xyz[2] = xyz[2]*Math.cos(yaw) + xyz[0]*Math.sin(yaw);
		xyz[0] = temp;
		//Yaw done
		
		//Do pitch:
		temp = xyz[0]*Math.cos(pitch) + xyz[1]*Math.sin(pitch); //Remember, +y is DOWN!!!
		xyz[1] = xyz[1]*Math.cos(pitch) - xyz[0]*Math.sin(pitch);
		xyz[0] = temp;
		//Pitch done (though possibly incorrect)
		
		//Do roll:
		temp = xyz[1]*Math.cos(roll) + xyz[2]*Math.sin(roll);
		xyz[2] = xyz[2]*Math.cos(roll) - xyz[1]*Math.sin(roll);
		xyz[1] = temp;
		//Roll done
	}
	
	private void drawSide(Graphics2D g2d, double[]i1, double[]i2){
		//Assumes the input values are relative to the upper left corner
		
		double scalescale = 0.1; //Should always be less than 1!!!
		
		//NO ALIASING!!!
		double[] p1 = Arrays.copyOf(i1, 3);
		double[] p2 = Arrays.copyOf(i2, 3);
		
		//Scale x1,y1 relative to z1
		p1[2] /= sideLength; //Get z between +/-sqrt(3)
		p1[2] *= scalescale; //Scale that interval if needed
		p1[2] ++; //Shift that interval so it's centered at 1
		p1[0]*=p1[2]; p1[1]*=p1[2]; //If larger z, scale the x and y so it looks like it's in perspective
		
		//Scale x2,y2 relative to z2
		p2[2] /= sideLength;
		p2[2] *= scalescale; //Scale that interval if needed
		p2[2] ++; //Shift that interval so it's centered at 1
		p2[0]*=p2[2]; p2[1]*=p2[2]; //Scale x and y
		
		//After scaling, center the drawing
		p1[0] += 0.5*frameWidth; p2[0] += 0.5*frameWidth; //Remember, +y is down...
		p1[1] += 0.5*frameHeight; p2[1] += 0.5*frameHeight;
		
		//Now actually draw it
		g2d.drawLine((int)p1[0], (int)p1[1], (int)p2[0], (int)p2[1]);
	}
	
	@Override
	public void paintComponent(Graphics g) {
		//Two bugs right now:
		//First, z-scaling isn't working. Instead of taking points in a cube and scaling them, I'm apparently rotating a 3-D trapezoid
		//Second, pitch is doing really weird stuff.
		
		setup();
		super.paintComponent(g);
		if (!setup){
			setup();
			setup = true;
		}
		Graphics2D g2d = (Graphics2D) g.create();
		
		g.drawImage(background, 0, 0, frameWidth, frameHeight, Color.black, null);
		
		//Fill vertices with the corners of a cube centered around the origin
		double[][] vertices = new double[8][3];
		int count = 0;
		for(double x = -0.5; x <= 0.5; x++){
			for(double y = -0.5; y <= 0.5; y++){
				for(double z = -0.5; z <= 0.5; z++){
					vertices[count][0] = x*sideLength;
					vertices[count][1] = y*sideLength;
					vertices[count][2] = z*sideLength;
					rotateYPR(vertices[count], yaw, pitch, roll); //Now aliases
					count++;
				}
			}
		}
		//I should now have correct rotated coordinates
		
		//Connections: 	0-1, 1-3, 3-2, 0-2
		//				4-5, 5-7, 7-6, 4-6
		//				0-4, 1-5, 2-6, 3-7
		
		//Draw the cube
		g2d.setColor(Color.white);
		drawSide(g2d, vertices[0], vertices[1]);
		drawSide(g2d, vertices[1], vertices[3]);
		drawSide(g2d, vertices[3], vertices[2]);
		drawSide(g2d, vertices[0], vertices[2]);
		
		drawSide(g2d, vertices[4], vertices[5]);
		drawSide(g2d, vertices[5], vertices[7]);
		drawSide(g2d, vertices[7], vertices[6]);
		drawSide(g2d, vertices[4], vertices[6]);

		drawSide(g2d, vertices[0], vertices[4]);
		drawSide(g2d, vertices[1], vertices[5]);
		drawSide(g2d, vertices[2], vertices[6]);
		drawSide(g2d, vertices[3], vertices[7]);
		
		//Only used for testing
		//yaw += 0.01;
		//pitch += 0.02;
		//roll += 0.03;
		
		g2d.dispose();
	}/**/
}