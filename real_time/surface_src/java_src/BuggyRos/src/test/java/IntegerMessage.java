

import com.roboclub.robobuggy.ros.Message;

public class IntegerMessage implements Message {

	public int val;
	private long sequenceNumber = -1;
	
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
        //autogen
    }

    @Override
    public String getTopicName() {
        return null;
    }

}
