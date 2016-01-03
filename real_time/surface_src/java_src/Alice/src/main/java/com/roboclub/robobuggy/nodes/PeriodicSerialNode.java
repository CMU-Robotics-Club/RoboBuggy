package com.roboclub.robobuggy.nodes;

import java.util.Timer;
import java.util.TimerTask;

import com.roboclub.robobuggy.ros.Node;

/**
 *
 * @author Trevor Decker
 *
 * @brif  This node will cause for an update function to be run at a fixed frequency
 *
 */
public abstract class PeriodicSerialNode extends SerialNode implements Node{

    private  final int RUN_PERIOD;

    /**
     * Create a new PeriodicSerialNode
     * @param name Name of the thread to read the serial messages
     * @param period of the periodically executed portion of the node
     */
    protected PeriodicSerialNode(String name, int period){
    	super(name);
        RUN_PERIOD = period;
        TimerTask timerTask = new UpdateTask();
        Timer timer = new Timer(true);
        timer.scheduleAtFixedRate(timerTask, 0, RUN_PERIOD);
    }
    
    /**
     * Method run periodically by the PeriodicSerialNode
     */
    abstract protected void update();

    /**
     * Task used to call the update method
     */
	private class UpdateTask extends TimerTask{
	
	        @Override
	        public void run() {
	                update();
	        }
	}
}