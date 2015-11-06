package c_two_source_integrator.startercode;

import com.roboclub.robobuggy.ros.Message;

public class IntegerMessage implements Message {

	int val;
	
	public IntegerMessage(int i) {
		this.val = i;
	}
	
	@Override
	public String toLogString() {
		return Integer.toString(this.val);
	}

	@Override
	public Message fromLogString(String str) {
		return new IntegerMessage(Integer.parseInt(str));
	}

}
