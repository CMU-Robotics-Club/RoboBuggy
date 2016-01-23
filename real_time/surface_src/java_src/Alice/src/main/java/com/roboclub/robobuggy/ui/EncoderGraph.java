package com.roboclub.robobuggy.ui;
import java.util.Random;

import javax.swing.JPanel;

import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.ros.Subscriber;

import org.jfree.chart.*;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
public class EncoderGraph {
	
	private XYSeriesCollection collection; //contains more than one series of data
	                                      // series are all indexed in the collection
	private XYSeries distanceData;
	//could probably create more sets of data for other stuff like steering, velocity etc.
	public EncoderGraph()
	{
		collection = new XYSeriesCollection();
		
		distanceData = new XYSeries("Distance Data");
        new Subscriber(SensorChannel.ENCODER.getMsgPath(), new MessageListener() {
			public void actionPerformed(String topicName, Message m) {
				EncoderMeasurement em = (EncoderMeasurement)m;
				double val2 = em.distance;
				double val1 = em.timestamp.getTime();
		        distanceData.add(val1, val2);
		    }
		});
	}
	
	public void makeDistanceGraph()
	{
		collection.addSeries(distanceData);
		JFreeChart chart = ChartFactory.createScatterPlot("Test Scatter PLot", "X values", "Y values",
	    		   collection, PlotOrientation.HORIZONTAL, true, true, false);
	       ChartFrame frame = new ChartFrame("Test", chart);
	       frame.pack();
	       frame.setVisible(true);
	}

}
