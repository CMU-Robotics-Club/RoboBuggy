import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.util.Date;
import java.text.DateFormat;
import java.text.SimpleDateFormat;

import java.awt.Color;

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
import org.jfree.ui.ApplicationFrame;
import org.jfree.ui.RefineryUtilities;

public class GUI extends JFrame
{
	//default value gets reset in the constructor based on the screen size 
	private int WIDTH = 400;
	private int HEIGHT = 300; 
	
	//Button handler
	private ExitButtonHandler ebHandler;
	private timerHandler tHandler;
	private javax.swing.Timer timer;
	
	//gui objects
	private JButton startPause_btn;
	private JLabel time_lbl;
	
	//global state
	private boolean playPauseState = false;
	
	
	public GUI()
	{
		//size of the screen
		Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();		
		WIDTH = screenSize.width;
		HEIGHT = screenSize.height;	
		
		setSize(WIDTH, HEIGHT);
		setVisible(true);
		setDefaultCloseOperation(EXIT_ON_CLOSE);
		
		setTitle("Data logging GUI");
		Container mainFrame = getContentPane();
		mainFrame.setLayout(new GridLayout(2, 2));			
		addCamPane(mainFrame);
		addDataLoggingPane(mainFrame);
		addGPSPane(mainFrame);
		addOtherDataPane(mainFrame);
		

	}
	
	private void addCamPane(Container parentFrame){
		JPanel camPanel = new JPanel();
		JLabel cam_message = new JLabel("Cam STUFF goes here",SwingConstants.CENTER);
		camPanel.add(cam_message);
		parentFrame.add(camPanel);
	}
	
	private void addDataLoggingPane(Container parentFrame){
		//stuff for setting up logging ie start/stop, file name ...
		JPanel dataLoggingPanel = new JPanel();
		dataLoggingPanel.setLayout(new GridLayout(4, 1));
		startPause_btn = new JButton("Start");
		playPauseState = true;
		startPause_btn.setFont(new Font("serif", Font.PLAIN, 70));
		startPause_btn.setBackground(Color.GREEN);
		StartPauseButtonHandler startPauseHandler = new StartPauseButtonHandler();
		startPause_btn.addActionListener(startPauseHandler);
		JLabel currentFile_lbl = new JLabel("currentFile",SwingConstants.CENTER);
		JLabel newFile_lbl = new JLabel("newFile",SwingConstants.CENTER);
		Date dateobj = new Date();
		DateFormat df = new SimpleDateFormat("dd/MM/yy HH:mm:ss");
	    System.out.println("");

		//TODO Date logTime = 
		
		time_lbl = new JLabel("SystemTime: " + df.format(dateobj) + " logTime: ",SwingConstants.CENTER);
		timer = new Timer(10, tHandler);//updates every .01 seconds
		timer.setInitialDelay(200); //waits .2 seconds to start for first time
		timer.start();
		dataLoggingPanel.add(startPause_btn);
		dataLoggingPanel.add(currentFile_lbl);
		dataLoggingPanel.add(newFile_lbl);
		dataLoggingPanel.add(time_lbl);
		parentFrame.add(dataLoggingPanel);
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
	
	
	private void addGPSPane(Container parentFrame){
		JPanel gpsPanel = new JPanel();
		JLabel gps_message = new JLabel("GPS STUFF goes here",SwingConstants.CENTER);
		gpsPanel.add(gps_message);		
		parentFrame.add(gpsPanel);

	}
	
	private class timerHandler implements ActionListener
	{

		@Override
		public void actionPerformed(ActionEvent e) {
				System.out.println("hello world\n");			
		}
		
		
	}
	
	private class StartPauseButtonHandler implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			//inverts the state of the system every time the button is pressed 
			playPauseState = !playPauseState;
			if(playPauseState)
			{	
				startPause_btn.setBackground(Color.RED);
				startPause_btn.setText("Pause");
			}else
			{	
				startPause_btn.setBackground(Color.GREEN);
				startPause_btn.setText("Start");
			}
			repaint();
		}
	}
	
	public class ExitButtonHandler implements ActionListener
	{
		public void actionPerformed(ActionEvent e)
		{
			System.exit(0);
		}
	}
	
	public static void main(String[] args)
	{
		GUI rectObj = new GUI();
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

           
       