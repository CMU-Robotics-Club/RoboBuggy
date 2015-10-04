package com.roboclub.robobuggy.calculatedNodes;

public abstract class BaseCalculatedNode {
	protected NodeCalculator calc;
	protected int delay;
	protected boolean enabled = true;
	protected long elapsed = 0;
	protected long start = System.currentTimeMillis();
	
	//TODO: Change this to a threadsafe implementation
	//TODO: Make this work if the object is still being initialized?
	public void enable() {
		this.enabled = true;
	}
	
	public void disable() {
		this.enabled = false;
	}
	
	abstract public void crunch();
}