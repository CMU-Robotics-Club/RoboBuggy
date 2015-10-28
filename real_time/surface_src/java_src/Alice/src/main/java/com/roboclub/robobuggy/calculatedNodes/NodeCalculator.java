package com.roboclub.robobuggy.calculatedNodes;

import com.roboclub.robobuggy.messages.BaseMessage;

public interface NodeCalculator {
	
	//Should be a function which returns time based measurements
	public BaseMessage calculator(long millis);
}
