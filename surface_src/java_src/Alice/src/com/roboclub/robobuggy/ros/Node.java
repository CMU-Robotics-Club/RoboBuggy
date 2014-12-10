package com.roboclub.robobuggy.ros;

public interface Node {
	
	boolean start(float hz);
	
	boolean shutdown();
	
	void spinOnce();
}
