package com.roboclub.robobuggy.calculatedNodes;

import com.roboclub.robobuggy.messages.EncoderMeasurement;

public class SimpleEncoderCalculator implements NodeCalculator {

	@Override
	public EncoderMeasurement calculator(long millis) {
		return new EncoderMeasurement(42, 42);
		// TODO Auto-generated method stub
	}
	
}
