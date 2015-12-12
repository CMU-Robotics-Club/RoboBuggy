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
public abstract class PeriodicNode implements Node{
        private  final int RUN_PERIOD;

        //period is in miliseconds 
        protected PeriodicNode(int period){
                RUN_PERIOD = period;
                TimerTask timerTask = new UpdateTask();
                Timer timer = new Timer(true);
                timer.scheduleAtFixedRate(timerTask, 0, RUN_PERIOD);
        }
        abstract protected void update();

private class UpdateTask extends TimerTask{

        @Override
        public void run() {
                update();
        }

}
}