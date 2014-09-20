package main;

import java.awt.Color;
import java.awt.GridLayout;

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

public class ImuPanel extends SerialPanel {
	private static final long serialVersionUID = -929040896215455343L;
	private static final char[] HEADER = {'a'};
	private static final int HEADER_LEN = 1;

	public ImuPanel() throws Exception {
		super("ARDUINO", 9600, HEADER, HEADER_LEN);
		this.setLayout(new GridLayout(3, 1));			

		//odom
		final XYDataset dataset = createDataset();
        final JFreeChart chart = createOdomChart(dataset);
        final ChartPanel odomChartPanel = new ChartPanel(chart);
//        odomChartPanel.setPreferredSize(new java.awt.Dimension(200, 200));
        this.add(odomChartPanel);
       
        //imu_rotX
        final XYDataset dataset1 = createDataset();
        final JFreeChart chart1 = createIMURotXChart(dataset);
        final ChartPanel imuRotXChartPanel = new ChartPanel(chart);
//        imuRotXChartPanel.setPreferredSize(new java.awt.Dimension(200, 270));
        this.add(imuRotXChartPanel);
        
        //comand_angle
        final XYDataset  dataset2 = createDataset();
        final JFreeChart chart2 = createCommandAngleChart(dataset);
        final ChartPanel commandAngleChartPanel = new ChartPanel(chart);
        this.add(commandAngleChartPanel);
	}
	
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

	    // change the auto tick unit selection to integer units only...
	    final NumberAxis rangeAxis = (NumberAxis) plot.getRangeAxis();
	    rangeAxis.setStandardTickUnits(NumberAxis.createIntegerTickUnits());
	    // OPTIONAL CUSTOMISATION COMPLETED.
	            
	    return chart;
	    
	}
}