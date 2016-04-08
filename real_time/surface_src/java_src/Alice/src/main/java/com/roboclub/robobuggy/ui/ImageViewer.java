package com.roboclub.robobuggy.ui;

import com.roboclub.robobuggy.messages.ImageMessage;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.Subscriber;

import javax.imageio.ImageIO;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

/**
 * 
 * @author Trevor Decker
 *
 *  A robobuggy gui element for viewing a stream from a camera 
 * 
 */
public class ImageViewer extends RobobuggyGUIContainer{
	private BufferedImage img = null;
	private Subscriber imageSub;

	/**
	 * Constructor for this object to view images 
	 * @param topicToSub the topic that images will be published on 
	 */
	public ImageViewer(String topicToSub){
		imageSub = new Subscriber("", topicToSub, new MessageListener() {
			
			@Override
			/**
			 * @param topicName that the action occurred on
			 * @param Message m that is received
			 */
			public void actionPerformed(String topicName, Message m) {
				ImageMessage imgM = (ImageMessage)m;
				img = imgM.getImage();
				repaint();
			}
		});
        String path = "images/rc_logo.png";
        File file = new File(path);
		try {
			img = ImageIO.read(file);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	@Override
	public void paintComponent(Graphics g) {
		super.paintComponent(g);
		Graphics2D g2d = (Graphics2D) g.create();
		g.drawImage(img, 10, 10, this.getWidth()-20, this.getHeight()-20, Color.black, null);		
		g2d.dispose();
	}

	

}
