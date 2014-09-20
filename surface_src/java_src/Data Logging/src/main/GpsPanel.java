package main;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;
import javax.swing.ImageIcon;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.SwingConstants;

public class GpsPanel extends SerialPanel {
	private static final long serialVersionUID = 1399590586061060311L;
	private static final char[] HEADER = {'#', 'A', 'C', 'G'};
	private static final int HEADER_LEN = 4;

	public GpsPanel() throws Exception {
		super("GPS", 57600, HEADER, HEADER_LEN);
		
		JLabel gps_message = new JLabel("GPS STUFF goes here",SwingConstants.CENTER);
		this.add(gps_message);
		
		/*BufferedImage myPicture;
		try {
			myPicture = ImageIO.read(new File("path-to-file"));
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			throw new RuntimeException("Course Map imqage could not be found");
		}
		JLabel picLabel = new JLabel(new ImageIcon(myPicture));
		add(picLabel);*/
	}
}