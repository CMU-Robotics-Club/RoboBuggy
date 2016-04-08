import java.util.concurrent.Semaphore;
import java.util.concurrent.atomic.AtomicInteger;

import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.Subscriber;

public class UnboundedQueueNumberSink implements Node {

	private Subscriber s;
	private Semaphore sem = new Semaphore(1);
	private AtomicInteger count = new AtomicInteger(1);

	public UnboundedQueueNumberSink(String topicName, int numMessages) {
		try {
			sem.acquire();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		s = new Subscriber("unboundedSink", topicName, new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				IntegerMessage im = (IntegerMessage) m;
				int val = count.getAndIncrement();

				if (val != im.val) {
					System.out.println("For an unbounded queue, the sum of dropped"
							+ " messages and our counter must equal the published messages.");
				}

				if ((val + 1) == numMessages) {
					sem.release();
					System.out.println("NumSink exiting...");
					return;
				}
			}
		});
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
		return count.get();
	}

}
