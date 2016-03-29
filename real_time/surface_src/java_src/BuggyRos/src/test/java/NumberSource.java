
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.Publisher;

public class NumberSource implements Node {

	Publisher p;

	boolean shouldShutdown = false;
	Thread pubThread;

	public NumberSource(String topic, int hz, int numMessages) {
		p = new Publisher(topic);

		pubThread = (new Thread(new Runnable() {
			int counter = 1;

			@Override
			public void run() {
				while (true) {
					IntegerMessage im = new IntegerMessage(counter);
					p.publish(im);
					counter++;

					if (counter == numMessages) {
						System.out.println("NumSource exiting...");
						return;
					}

					if (hz != 0) {
						try {
							Thread.sleep(1 / (1000 * hz));
						} catch (InterruptedException e) {
							// We are shutting down.
							return;
						}
					}
				}
			}
		}));

		pubThread.start();
	}

	@Override
	public boolean shutdown() {
		shouldShutdown = true;
		try {
			pubThread.join(500);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		pubThread.interrupt();

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

}
