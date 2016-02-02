package com.roboclub.robobuggy.ui;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;
import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;

import com.roboclub.robobuggy.messages.ImageMessage;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyNode;
import com.roboclub.robobuggy.nodes.baseNodes.PeriodicNode;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.Subscriber;

public class ImageViewer extends RobobuggyGUIContainer{
	BufferedImage img = null;
	Subscriber imageSub;

	
	public ImageViewer(String topic_to_sub){
		imageSub = new Subscriber(topic_to_sub, new MessageListener() {
			
			@Override
			public void actionPerformed(String topicName, Message m) {
				ImageMessage img_m = (ImageMessage)m;
				img = img_m.getImage();
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
		g.drawImage(img, 0, 0, this.getWidth(), this.getHeight(), Color.black, null);		
		g2d.dispose();
	}

	

}
