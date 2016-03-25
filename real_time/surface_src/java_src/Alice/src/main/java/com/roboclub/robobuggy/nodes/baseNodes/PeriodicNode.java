package com.roboclub.robobuggy.nodes.baseNodes;

import java.util.Timer;
import java.util.TimerTask;

/**
 * This node will cause for an update function to be run at a fixed frequency
 *
 * @author Zachary Dawson
 * @author Trevor Decker
 *
 */
public abstract class PeriodicNode extends BuggyDecoratorNode {

    private final int runPeriod;
    private TimerTask timerTask;
    private  Timer timer;

    /**
     * Create a new {@link PeriodicNode} decorator
     * @param base {@link BuggyNode} to decorate
     * @param period of the periodically executed portion of the node
     * Note the timer needs to be started by running resume
     */
    protected PeriodicNode(BuggyNode base, int period){
    	super(base);
        runPeriod = period;
        timerTask = new UpdateTask();
        pause();
    }
    
    protected synchronized void pause(){
    	if(timer != null){
    		timer.cancel();
    	}
    	
    }
    
    protected synchronized void resume(){
    	timer = new Timer(true);
        timer.scheduleAtFixedRate(timerTask, 0, runPeriod);
    	
    }
    
    /**
     * Method run periodically by the {@link PeriodicNode}
     */
    protected abstract void update();

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