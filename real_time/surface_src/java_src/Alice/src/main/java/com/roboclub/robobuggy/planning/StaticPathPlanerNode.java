package com.roboclub.robobuggy.planning;

import com.roboclub.robobuggy.nodes.PeriodicNode;


/**
 * 
 * @author Trevor Decker
 * This node outputs a static path for the buggy to attempt to follow 
 * the output is done periodically at a fixed frequency.  This is so that 
 * the path planning interface can be followed.  In other nodes the path 
 * that is being output may vary with time and other messages that
 *  are being output
 */
public class StaticPathPlanerNode extends PeriodicNode{

	//will resend the path every 1 second by default 
	StaticPathPlanerNode(){
		super(1000);
	}
	
	StaticPathPlanerNode(int period) {
		super(period);
	}

	@Override
	public boolean shutdown() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	protected void update() {
		//TODO send path plan message
				
	}

}
