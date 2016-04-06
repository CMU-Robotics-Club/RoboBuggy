import java.util.concurrent.Semaphore;
import java.util.concurrent.atomic.AtomicInteger;

import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.Subscriber;

public class BoundedQueueNumberSink implements Node {

	private Subscriber s;
	private Semaphore sem = new Semaphore(1);
	private AtomicInteger count = new AtomicInteger(1);

	public BoundedQueueNumberSink(String topicName, int numMessages) {
		this(topicName, numMessages, Integer.MAX_VALUE);
	}
	
	public BoundedQueueNumberSink(String topicName, int numMessages, int maxMessageQueueLength) {
		try {
			sem.acquire();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		s = new Subscriber("boundedSink", topicName, maxMessageQueueLength, new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				IntegerMessage im = (IntegerMessage) m;
				int val = count.getAndIncrement();
				
				if ((val + s.getNumberMessagesDropped())!= im.val) {
					System.out.println("For an bounded queue, the sum of dropped"
							+ " messages and our counter must equal the published messages.");
				}
				
				if ((val+s.getNumberMessagesDropped()+1) == numMessages) {
					sem.release();
					System.out.println("NumSink exiting...");
					return;
				}
			}
		});
	}
	
	public long getNumberMessagesDropped() {
		return s.getNumberMessagesDropped();
	}

	@Override
	public boolean shutdown() {
		s.close();
		return true;
	}

	@Override
	public boolean startNode() {
		return false;
	}

	@Override
	public void setName(String newName) {
		System.out.println("tried to set name");
	}

	@Override
	public String getName() {
		return null;
	}

	public int blockUntilDone() {
		try {
			sem.acquire();
		} catch (InterruptedException e) {
			assert (false);
		}
		return count.get() + (int)getNumberMessagesDropped();
	}

}
