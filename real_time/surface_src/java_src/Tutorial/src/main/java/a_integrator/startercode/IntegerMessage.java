package a_integrator.startercode;

import com.roboclub.robobuggy.ros.Message;

public class IntegerMessage implements Message {

	int val;
	long sequenceNumber;
	
	public IntegerMessage(int i) {
		this.val = i;
	}

	@Override
	public long getSequenceNumber() {
		return sequenceNumber;
	}

	@Override
	public void setSequenceNumber(long sequenceNumber) {
		this.sequenceNumber = sequenceNumber;
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
