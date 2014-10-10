package com.roboclub.robobuggy.ui;

import java.awt.Color;
import java.util.Date;

import javax.swing.BoxLayout;
import javax.swing.JPanel;

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

import com.roboclub.robobuggy.logging.RobotLogger;
import com.roboclub.robobuggy.serial.SerialEvent;
import com.roboclub.robobuggy.serial.SerialListener;

public class ImuPanel extends JPanel {
	private static final long serialVersionUID = 8766665153831557631L;
	
	private float aX;
	private float aY;
	private float aZ;
	private float rX;
	private float rY;
	private float rZ;
	private float mX;
	private float mY;
	private float mZ;
	
	//data stored for plotting
	private static final int HISTORY_LENGTH = 20; 
	private int count;
	private XYSeries aX_history;
	private XYSeries aY_history;
	private XYSeries aZ_history;
	private XYSeries rX_history;
	private XYSeries rY_history;
	private XYSeries rZ_history;
	private XYSeries mX_history;
	private XYSeries mY_history;
	private XYSeries mZ_history;
	
	public ImuPanel() {
		this.setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));

		//sensor history for display
		aX_history = new XYSeries("First");
		aY_history = new XYSeries("Second");
		aZ_history = new XYSeries("Thirty");
		rX_history = new XYSeries("First");
		rY_history = new XYSeries("Second");
		rZ_history = new XYSeries("Thirty");
		mX_history = new XYSeries("First");
		mY_history = new XYSeries("Second");
		mZ_history = new XYSeries("Thrty");
		count = 0;
		
		//odom
		final XYDataset dataset = createDataset();
        final JFreeChart chart = createOdomChart(dataset);
        final ChartPanel odomChartPanel = new ChartPanel(chart);
//        odomChartPanel.setPreferredSize(new java.awt.Dimension(200, 200));
        this.add(odomChartPanel);
       
        //imu_rotX
        final XYDataset dataset1 = createDataset();
        final JFreeChart chart1 = createIMURotXChart(dataset1);
        final ChartPanel imuRotXChartPanel = new ChartPanel(chart1);
//        imuRotXChartPanel.setPreferredSize(new java.awt.Dimension(200, 270));
        this.add(imuRotXChartPanel);
        
        //comand_angle
        final XYDataset  dataset2 = createDataset();
        final JFreeChart chart2 = createCommandAngleChart(dataset2);
        final ChartPanel commandAngleChartPanel = new ChartPanel(chart2);
        this.add(commandAngleChartPanel);
	}
	
	private void addToDisplay() {
		addToHistory(aX_history,Double.valueOf(aX));
	    addToHistory(aY_history,Double.valueOf(aY));
	    addToHistory(aZ_history,Double.valueOf(aZ));
	    addToHistory(rX_history,Double.valueOf(rX));
	    addToHistory(rY_history,Double.valueOf(rY));
	    addToHistory(rZ_history,Double.valueOf(rZ));
	    addToHistory(mX_history,Double.valueOf(mX));
	    addToHistory(mY_history,Double.valueOf(mY));
	    addToHistory(mZ_history,Double.valueOf(mZ));
	    
	    count++;
	}
	
	private void addToHistory(XYSeries history,double newdata) {
	    if(history.getItemCount() > HISTORY_LENGTH) {
	    	history.remove(0);
	    }
	    history.add(count,newdata);
	}
		
	XYDataset createDataset() {

	    final XYSeriesCollection dataset = new XYSeriesCollection();
	    dataset.addSeries(aX_history);
	    dataset.addSeries(aY_history);
	    dataset.addSeries(aZ_history);
	            
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

	// final StandardLegend legend = (StandardLegend) chart.getLegend();
	//  legend.setDisplaySeriesShapes(true);
	 
	 // get a reference to the plot for further customisation...
	 final XYPlot plot = chart.getXYPlot();
	 plot.setBackgroundPaint(Color.lightGray);
//	    plot.setAxisOffset(new Spacer(Spacer.ABSOLUTE, 5.0, 5.0, 5.0, 5.0));
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

	// final StandardLegend legend = (StandardLegend) chart.getLegend();
	//  legend.setDisplaySeriesShapes(true);
	 
	 // get a reference to the plot for further customisation...
	 final XYPlot plot = chart.getXYPlot();
	 plot.setBackgroundPaint(Color.lightGray);
//	    plot.setAxisOffset(new Spacer(Spacer.ABSOLUTE, 5.0, 5.0, 5.0, 5.0));
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
	        "Omom ticks",      // chart title
	        "time",                      // x axis label
	        "mag",                      // y axis label
	        dataset,                  // data
	        PlotOrientation.VERTICAL,
	        true,                     // include legend
	        true,                     // tooltips
	        false                     // urls
	    );

	    // NOW DO SOME OPTIONAL CUSTOMISATION OF THE CHART...
	    chart.setBackgroundPaint(Color.white);

//	    final StandardLegend legend = (StandardLegend) chart.getLegend();
//	      legend.setDisplaySeriesShapes(true);
	    
	    // get a reference to the plot for further customisation...
	    final XYPlot plot = chart.getXYPlot();
	    plot.setBackgroundPaint(Color.lightGray);
//	    plot.setAxisOffset(new Spacer(Spacer.ABSOLUTE, 5.0, 5.0, 5.0, 5.0));
	    plot.setDomainGridlinePaint(Color.white);
	    plot.setRangeGridlinePaint(Color.white);
	    
	    final XYLineAndShapeRenderer renderer = new XYLineAndShapeRenderer();
	    renderer.setSeriesLinesVisible(0, false);
	    renderer.setSeriesShapesVisible(1, false);
	    plot.setRenderer(renderer);

	    NumberAxis range = (NumberAxis) plot.getRangeAxis();
        range.setRange(0.0, 20.0);
	    // change the auto tick unit selection to integer units only...
	    final NumberAxis rangeAxis = (NumberAxis) plot.getRangeAxis();
	    rangeAxis.setStandardTickUnits(NumberAxis.createIntegerTickUnits());
	    // OPTIONAL CUSTOMISATION COMPLETED.
	         
	    return chart;
	    
	}
}