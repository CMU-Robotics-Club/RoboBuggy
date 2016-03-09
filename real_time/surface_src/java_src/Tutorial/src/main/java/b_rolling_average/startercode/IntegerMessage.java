package b_rolling_average.startercode;

import com.roboclub.robobuggy.ros.Message;

public class IntegerMessage implements Message {

	int val;
	
	public IntegerMessage(int i) {
		this.val = i;
	}

}
