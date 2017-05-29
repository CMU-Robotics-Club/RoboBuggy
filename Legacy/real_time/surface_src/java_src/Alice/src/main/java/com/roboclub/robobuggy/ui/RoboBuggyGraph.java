package com.roboclub.robobuggy.ui;

import com.roboclub.robobuggy.main.RobobuggyConfigFile;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.Subscriber;
import com.sun.javafx.geom.Vec2d;
import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

import java.util.ArrayList;

/**
 * a graph
 */
public class RoboBuggyGraph extends RobobuggyGUIContainer {
    private ArrayList<Vec2d> list = new ArrayList();
    private ChartPanel chartPanel;
    private JFreeChart chart;

    /**
     * returns a new point in x and y coords
     */
    public interface GetGraphValues {
        /**
         * @param m message to derive x and y from
         * @return the x coord of the new point to add to the graph
         */
        double getX(Message m);

        /**
         * @param m message to derive x and y from
         * @return the y coord of the new point to add to the graph
         */
        double getY(Message m);
    }


    /**
     * makes a new imurollgraph
     *
     * @param func  the function that gets the points for the graph
     * @param title title of the graph
     * @param topic topic for the subscriber
     */
    public RoboBuggyGraph(String title, String topic, GetGraphValues func) {
        XYSeries series1 = new XYSeries(title);

        XYSeriesCollection dataset = new XYSeriesCollection();
        dataset.addSeries(series1);


        new Subscriber("uiBuggy", topic, new MessageListener() {

            @Override
            public void actionPerformed(String topicName, Message m) {

                while (series1.getItemCount() > RobobuggyConfigFile.GRAPH_LENGTH) {
                    //TODO add a lock
                    series1.remove(0);
                }

                series1.add(func.getX(m), func.getY(m));

            }
        });

        chart = ChartFactory.createXYLineChart(title, "xAxisLabel", "yAxisLabel", dataset,
                PlotOrientation.VERTICAL, true, true, true);
        chartPanel = new ChartPanel(chart);
        add(chartPanel);

        Thread thread = new Thread() {
            public void run() {
                while (true) {
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

