package com.roboclub.robobuggy.ui;


import java.util.ArrayList;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

import com.roboclub.robobuggy.main.RobobuggyConfigFile;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Subscriber;
import com.sun.javafx.geom.Vec2d;



/**
 * a graph
 */
public class RoboBuggyGraph extends RobobuggyGUIContainer{
	private ArrayList<Vec2d> list = new ArrayList();
	private ChartPanel chartPanel;
	private JFreeChart chart;
	
	public interface getGraphValues {
		public double getX(Message m);
		public double getY(Message m);

	}
	

	/**
	 * makes a new imurollgraph
	 */
	public RoboBuggyGraph(String title,String topic,getGraphValues func){
		XYSeries series1 = new XYSeries(title);
		
		XYSeriesCollection dataset = new XYSeriesCollection();
		dataset.addSeries(series1);

		

		new Subscriber(topic, new MessageListener() {
		
			@Override
			public void actionPerformed(String topicName, Message m) {
				
				while(series1.getItemCount() > RobobuggyConfigFile.GRAPH_LENGTH){
					series1.remove(0);
				}
				
				func.getY(m);
				series1.add(func.getX(m), func.getY(m));
		
			}
		});
		
		chart = ChartFactory.createXYLineChart(title, "xAxisLabel", "yAxisLabel", dataset,
				PlotOrientation.VERTICAL, true, true, true);
		chartPanel = new ChartPanel(chart);			
		add(chartPanel);

		  Thread thread = new Thread(){
			    public void run(){
			    	while(true){
			    		//TODO get sizing to be better 
			    		JFreeChart chart = ChartFactory.createXYLineChart(title, "xAxisLabel", "yAxisLabel",
								dataset, PlotOrientation.VERTICAL, true, true, true);
			    		chartPanel = new ChartPanel(chart);	
			    		try {
			    			this.sleep(1000);
			    		} catch (InterruptedException e) {
			    			// TODO Auto-generated catch block
			    			e.printStackTrace();
			    		}
			    	}
					}
			  };
			  
			  thread.start();

		
		
		

	}
	
	
  

}

