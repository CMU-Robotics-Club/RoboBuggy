package com.roboclub.robobuggy.ros;

public interface Node {
	String name = "";
	boolean shutdown();
	
	default String getName() {
		// TODO Auto-generated method stub
		return this.name;
	}
	
}