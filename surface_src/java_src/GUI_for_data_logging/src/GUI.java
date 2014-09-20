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
	
	private static VideoCapture camera;
	
	private static ArduinoPanel arduinoPanel;
	private static CameraPanel cameraPanel;
	private static GpsPanel gpsPanel;
	private static ImuPanel imuPanel;
	
	public GUI()
	{
		//size of the screen
		Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();		
		WIDTH = screenSize.width-2;
		HEIGHT = screenSize.height-2;	
		
		setSize(WIDTH, HEIGHT);
		setVisible(true);
		setDefaultCloseOperation(EXIT_ON_CLOSE);

		// Initialize panels
		arduinoPanel = new ArduinoPanel("COM5", 9600);
		cameraPanel = new CameraPanel(0);
		gpsPanel = new GpsPanel();
		imuPanel = new ImuPanel("COM10", 57600);
		
		setTitle("Data logging GUI");
		Container mainFrame = getContentPane();
		mainFrame.setLayout(new GridLayout(2, 2));			
		mainFrame.add()
		addCamPane(mainFrame);
		addDataLoggingPane(mainFrame);
		addGPSPane(mainFrame);
		addOtherDataPane(mainFrame);
	}
	
	private void addOtherDataPane(Container parentFrame){
			//encoders and IMU data 

			JPanel otherDataPanel = new JPanel();
		    otherDataPanel.setLayout(new GridLayout(3, 1));			


			//odom
			final XYDataset dataset = createDataset();
	        final JFreeChart chart = createOdomChart(dataset);
	        final ChartPanel odomChartPanel = new ChartPanel(chart);
	//        odomChartPanel.setPreferredSize(new java.awt.Dimension(200, 200));
	        otherDataPanel.add(odomChartPanel);
	       
	        //imu_rotX
	        final XYDataset dataset1 = createDataset();
	        final JFreeChart chart1 = createIMURotXChart(dataset);
	        final ChartPanel imuRotXChartPanel = new ChartPanel(chart);
//	        imuRotXChartPanel.setPreferredSize(new java.awt.Dimension(200, 270));
	        otherDataPanel.add(imuRotXChartPanel);
	        
	        //comand_angle
	        final XYDataset  dataset2 = createDataset();
	        final JFreeChart chart2 = createCommandAngleChart(dataset);
	        final ChartPanel commandAngleChartPanel = new ChartPanel(chart);
	        otherDataPanel.add(commandAngleChartPanel);
	
			parentFrame.add(otherDataPanel);

		}
	
	
	private class timerHandler implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent ae) {
				System.out.println("hello world\n");			
				repaint();
		}
	}
	
	
	
	public static void main(String[] args)
	{
		GUI rectObj = new GUI();
		
		// Initialize camera
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		try {
			initializeCamera(0);
		} catch (Exception e) {
			e.printStackTrace();
			return;
		}
		
		// Read from camera forever
		/*Mat frame = new Mat();
		while(true) {
			camera.read(frame);
			
			MatOfByte bytemat = new MatOfByte();
			Highgui.imencode(".jpg", frame, bytemat);
			byte[] bytes = bytemat.toArray();
			InputStream in = new ByteArrayInputStream(bytes);
			
			try {
				BufferedImage img = ImageIO.read(in);
				camPanel.add( new JLabel( new ImageIcon(img) ) );
				camPanel.setVisible(true);
			} catch (Exception e) {
				e.printStackTrace();
				return;
			}
		}*/
	}
	
	public static void initializeCamera(int camera_num) throws Exception {
		
		camera = new VideoCapture(camera_num);
		
		try {
			// Allow camera to initialize
			Thread.sleep(1000);	
		} catch (Exception e) {
			e.printStackTrace();
			throw new Exception("Unable to intialize camera object");
		}
		
		camera.open(0);
		if (!camera.isOpened()) {
			throw new Exception("Unable to Open Camera");
		}
	}
	



//import org.jfree.ui.Spacer;

/*
public class GUI extends ApplicationFrame {


    public GUI(final String title) {

        super(title);

        final XYDataset dataset = createDataset();
        final JFreeChart chart = createChart(dataset);
        final ChartPanel chartPanel = new ChartPanel(chart);
        chartPanel.setPreferredSize(new java.awt.Dimension(500, 270));
        setContentPane(chartPanel);

    }
    */
    
 XYDataset createDataset() {
        
        final XYSeries series1 = new XYSeries("First");
        series1.add(1.0, 1.0);
        series1.add(2.0, 4.0);
        series1.add(3.0, 3.0);
        series1.add(4.0, 5.0);
        series1.add(5.0, 5.0);
        series1.add(6.0, 7.0);
        series1.add(7.0, 7.0);
        series1.add(8.0, 8.0);

        final XYSeries series2 = new XYSeries("Second");
        series2.add(1.0, 5.0);
        series2.add(2.0, 7.0);
        series2.add(3.0, 6.0);
        series2.add(4.0, 8.0);
        series2.add(5.0, 4.0);
        series2.add(6.0, 4.0);
        series2.add(7.0, 2.0);
        series2.add(8.0, 1.0);

        final XYSeries series3 = new XYSeries("Third");
        series3.add(3.0, 4.0);
        series3.add(4.0, 3.0);
        series3.add(5.0, 2.0);
        series3.add(6.0, 3.0);
        series3.add(7.0, 6.0);
        series3.add(8.0, 3.0);
        series3.add(9.0, 4.0);
        series3.add(10.0, 3.0);

        final XYSeriesCollection dataset = new XYSeriesCollection();
        dataset.addSeries(series1);
        dataset.addSeries(series2);
        dataset.addSeries(series3);
                
        return dataset;
        
    }
    
 
 private JFreeChart createIMURotXChart(final XYDataset dataset){
	    
     // create the chart...
     final JFreeChart chart = ChartFactory.createXYLineChart(
         "IMU X rot",      // chart title
         "time (s)",                      // x axis label
         "radians",                      // y axis label
         dataset,                  // data
         PlotOrientation.VERTICAL,
         true,                     // include legend
         true,                     // tooltips
         false                     // urls
     );

     // NOW DO SOME OPTIONAL CUSTOMISATION OF THE CHART...
     chart.setBackgroundPaint(Color.white);

//     final StandardLegend legend = (StandardLegend) chart.getLegend();
//      legend.setDisplaySeriesShapes(true);
     
     // get a reference to the plot for further customisation...
     final XYPlot plot = chart.getXYPlot();
     plot.setBackgroundPaint(Color.lightGray);
 //    plot.setAxisOffset(new Spacer(Spacer.ABSOLUTE, 5.0, 5.0, 5.0, 5.0));
     plot.setDomainGridlinePaint(Color.white);
     plot.setRangeGridlinePaint(Color.white);
     
     final XYLineAndShapeRenderer renderer = new XYLineAndShapeRenderer();
     renderer.setSeriesLinesVisible(0, false);
     renderer.setSeriesShapesVisible(1, false);
     plot.setRenderer(renderer);

     // change the auto tick unit selection to integer units only...
     final NumberAxis rangeAxis = (NumberAxis) plot.getRangeAxis();
     rangeAxis.setStandardTickUnits(NumberAxis.createIntegerTickUnits());
     // OPTIONAL CUSTOMISATION COMPLETED.
             
     return chart;
     
 }

 


 private JFreeChart createCommandAngleChart(final XYDataset dataset) {
     
     // create the chart...
     final JFreeChart chart = ChartFactory.createXYLineChart(
         "Command Angle",      // chart title
         "time",                      // x axis label
         "degrees",                      // y axis label
         dataset,                  // data
         PlotOrientation.VERTICAL,
         true,                     // include legend
         true,                     // tooltips
         false                     // urls
     );

     // NOW DO SOME OPTIONAL CUSTOMISATION OF THE CHART...
     chart.setBackgroundPaint(Color.white);

//     final StandardLegend legend = (StandardLegend) chart.getLegend();
//      legend.setDisplaySeriesShapes(true);
     
     // get a reference to the plot for further customisation...
     final XYPlot plot = chart.getXYPlot();
     plot.setBackgroundPaint(Color.lightGray);
 //    plot.setAxisOffset(new Spacer(Spacer.ABSOLUTE, 5.0, 5.0, 5.0, 5.0));
     plot.setDomainGridlinePaint(Color.white);
     plot.setRangeGridlinePaint(Color.white);
     
     final XYLineAndShapeRenderer renderer = new XYLineAndShapeRenderer();
     renderer.setSeriesLinesVisible(0, false);
     renderer.setSeriesShapesVisible(1, false);
     plot.setRenderer(renderer);

     // change the auto tick unit selection to integer units only...
     final NumberAxis rangeAxis = (NumberAxis) plot.getRangeAxis();
     rangeAxis.setStandardTickUnits(NumberAxis.createIntegerTickUnits());
     // OPTIONAL CUSTOMISATION COMPLETED.
             
     return chart;
     
 }

 
	 
    private JFreeChart createOdomChart(final XYDataset dataset) {
        
        // create the chart...
        final JFreeChart chart = ChartFactory.createXYLineChart(
            "Line Chart Demo 6",      // chart title
            "X",                      // x axis label
            "Y",                      // y axis label
            dataset,                  // data
            PlotOrientation.VERTICAL,
            true,                     // include legend
            true,                     // tooltips
            false                     // urls
        );

        // NOW DO SOME OPTIONAL CUSTOMISATION OF THE CHART...
        chart.setBackgroundPaint(Color.white);

//        final StandardLegend legend = (StandardLegend) chart.getLegend();
  //      legend.setDisplaySeriesShapes(true);
        
        // get a reference to the plot for further customisation...
        final XYPlot plot = chart.getXYPlot();
        plot.setBackgroundPaint(Color.lightGray);
    //    plot.setAxisOffset(new Spacer(Spacer.ABSOLUTE, 5.0, 5.0, 5.0, 5.0));
        plot.setDomainGridlinePaint(Color.white);
        plot.setRangeGridlinePaint(Color.white);
        
        final XYLineAndShapeRenderer renderer = new XYLineAndShapeRenderer();
        renderer.setSeriesLinesVisible(0, false);
        renderer.setSeriesShapesVisible(1, false);
        plot.setRenderer(renderer);

        // change the auto tick unit selection to integer units only...
        final NumberAxis rangeAxis = (NumberAxis) plot.getRangeAxis();
        rangeAxis.setStandardTickUnits(NumberAxis.createIntegerTickUnits());
        // OPTIONAL CUSTOMISATION COMPLETED.
                
        return chart;
        
    }
    }

    // ****************************************************************************
    // * JFREECHART DEVELOPER GUIDE                                               *
    // * The JFreeChart Developer Guide, written by David Gilbert, is available   *
    // * to purchase from Object Refinery Limited:                                *
    // *                                                                          *
    // * http://www.object-refinery.com/jfreechart/guide.html                     *
    // *                                                                          *
    // * Sales are used to provide funding for the JFreeChart project - please    * 
    // * support us so that we can continue developing free software.             *
    // ****************************************************************************
    
/*
    public static void main(final String[] args) {

        final GUI demo = new GUI("Line Chart Demo ");
        demo.pack();
        RefineryUtilities.centerFrameOnScreen(demo);
        demo.setVisible(true);

    }

}
*/

           
       