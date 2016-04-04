package a_integrator.msebek;

import com.roboclub.robobuggy.ros.Message;

public class IntegerMessage implements Message {

	int val;
	
	public IntegerMessage(int i) {
		this.val = i;
	}

	@Override
	public void setTopicName(String name) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public String getTopicName() {
		// TODO Auto-generated method stub
		return null;
	}

}
