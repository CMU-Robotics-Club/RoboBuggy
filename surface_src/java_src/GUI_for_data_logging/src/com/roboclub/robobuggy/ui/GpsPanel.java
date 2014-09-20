package com.roboclub.robobuggy.ui;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;
import javax.swing.ImageIcon;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.SwingConstants;

public class GpsPanel extends JPanel {
	private static final long serialVersionUID = 1399590586061060311L;

	public GpsPanel() {
		JPanel gpsPanel = new JPanel();
		JLabel gps_message = new JLabel("GPS STUFF goes here",SwingConstants.CENTER);
		gpsPanel.add(gps_message);
		
		BufferedImage myPicture;
		try {
<<<<<<< HEAD
			myPicture = ImageIO.read(new File("map.jpg"));
			
=======
			System.out.println(System.getProperty("user.dir"));
			//RoboBuggy\surface_src\java_src\GUI_for_data_logging\
			myPicture = ImageIO.read(new File("mapTest.jpg")); //random image
>>>>>>> c9d596ffc3b4473a17ec59eca21d07280eac52ba
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			throw new RuntimeException("Course Map image could not be found");
		}
		JLabel picLabel = new JLabel(new ImageIcon(myPicture));
		add(picLabel);

	}
}