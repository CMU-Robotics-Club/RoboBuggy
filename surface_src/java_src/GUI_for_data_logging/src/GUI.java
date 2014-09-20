import java.awt.*;

import javax.imageio.ImageIO;
import javax.swing.*;

import java.awt.event.*;
import java.awt.image.BufferedImage;
import java.util.Date;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.awt.Color;
import java.io.ByteArrayInputStream;
import java.io.InputStream;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.XYLineAndShapeRenderer;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;

import com.roboclub.robobuggy.ui.ArduinoPanel;
import com.roboclub.robobuggy.ui.CameraPanel;
import com.roboclub.robobuggy.ui.ControlsPanel;
import com.roboclub.robobuggy.ui.GpsPanel;
import com.roboclub.robobuggy.ui.ImuPanel;

public class GUI extends JFrame
{
	private static final long serialVersionUID = 8821046675205954386L;
	
	// will reset these on init
	private int WIDTH = 400;
	private int HEIGHT = 300; 
	
	// Timer handler
	private javax.swing.Timer timer;
	
	// Lovely global state
	private boolean playPauseState = false;
	
	private static ArduinoPanel arduinoPanel;
	private static CameraPanel cameraPanel;
	private static ControlsPanel controlsPanel;
	private static GpsPanel gpsPanel;
	private static ImuPanel imuPanel;
	
	public GUI() throws Exception
	{
		//size of the screen
		Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();		
		WIDTH = screenSize.width-10;
		HEIGHT = screenSize.height-30;	
		
		setSize(WIDTH, HEIGHT);
		setVisible(true);
		setDefaultCloseOperation(EXIT_ON_CLOSE);

		// Initialize panels
		arduinoPanel = new ArduinoPanel("COM5", 9600);
		cameraPanel = new CameraPanel(0);
		gpsPanel = new GpsPanel();
		imuPanel = new ImuPanel("COM10", 57600);
		controlsPanel = new ControlsPanel();
		
		
		setTitle("RoboBuggy Data Gathering");
		Container mainFrame = getContentPane();
		mainFrame.setLayout(new GridLayout(2, 2));			
		mainFrame.add(cameraPanel);
		mainFrame.add(controlsPanel);
		mainFrame.add(gpsPanel);
		mainFrame.add(arduinoPanel);
		mainFrame.repaint();
		
		mainFrame = mainFrame;
		// TODO: add imuPanel
	}
	

	
	public static void main(String[] args) throws Exception
	{
		GUI rectObj = new GUI();
		
		// Initialize camera
		/*System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		try {
			initializeCamera(0);
		} catch (Exception e) {
			e.printStackTrace();
			return;
		}*/
		

	}
	
}
	
	
       