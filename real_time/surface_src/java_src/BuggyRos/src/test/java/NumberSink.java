import java.util.concurrent.Semaphore;
import java.util.concurrent.atomic.AtomicInteger;

import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.Subscriber;

public class NumberSink implements Node {

	Subscriber s;
	Semaphore sem = new Semaphore(1);
	AtomicInteger count = new AtomicInteger(1);

	public NumberSink(String topicName, int numMessages) {
		try {
			sem.acquire();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		s = new Subscriber(topicName, new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				IntegerMessage im = (IntegerMessage) m;
				int val = count.getAndIncrement();
				//System.out.println("message " + val + ":" + im.val);
				if ((val+1) == numMessages) {
					sem.release();
					System.out.println("NumSink exiting...");
					return;
				}
			}
		});
	}

	@Override
	public boolean shutdown() {
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
