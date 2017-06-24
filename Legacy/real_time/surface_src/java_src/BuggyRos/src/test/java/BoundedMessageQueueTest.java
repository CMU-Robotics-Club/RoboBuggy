import static org.junit.Assert.fail;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

public class BoundedMessageQueueTest {

	@Before
	public void setUp() throws Exception {
	}

	@After
	public void tearDown() throws Exception {
	}

	@Test
	public void onePubOneSub() {
		int numIterations = 1000000;
		BoundedQueueNumberSink sink = new BoundedQueueNumberSink("to", numIterations, 1);
	
		// Publish as fast as we can!
		NumberSource source = new NumberSource("to", 0, numIterations);

		System.out.println("Blocking on done...");
		int count = sink.blockUntilDone();
		if(count != numIterations) {
			fail("Counts did not match");
		}

		source.shutdown();
		sink.shutdown();
	}

	@Test
	public void onePubOneSubWithMiddleNode() {
		int numIterations = 1000000;
		BoundedQueueNumberSink sink = new BoundedQueueNumberSink("to", numIterations);
		MessagePasser ms = new MessagePasser("middle", "from", "to");
	
		// Publish as fast as we can!
		NumberSource source = new NumberSource("from", 0, numIterations);

		System.out.println("Blocking on done...");
		int count = sink.blockUntilDone();
		if(count != numIterations) {
			fail("Counts did not match");
		}
		
		sink.shutdown();
		ms.shutdown();
		source.shutdown();
	}

	public void onePubOneSubWithTwoMiddleNode(int localInboxLength) {
		int numIterations = 100000;
		BoundedQueueNumberSink sink = new BoundedQueueNumberSink("to", numIterations);
		MessagePasser ms1 = new MessagePasser("firstPasser", "from", "middle");
		MessagePasser ms2 = new MessagePasser("secondPasser", "middle", "to");
	
		// Publish as fast as we can!
		NumberSource source = new NumberSource("from", 0, numIterations);

		System.out.println("Blocking on done...");
		int count = sink.blockUntilDone();
		if(count != numIterations) {
			fail("Counts did not match");
		}

		sink.shutdown();
		ms1.shutdown();
		ms2.shutdown();
		source.shutdown();
	}

	// TODO: get stats on how many messages are being dropped
	@Test
	public void BoundedQueuesOfDifferentLengths() {
		onePubOneSubWithTwoMiddleNode(1);
		onePubOneSubWithTwoMiddleNode(10);
		onePubOneSubWithTwoMiddleNode(100);
		onePubOneSubWithTwoMiddleNode(1000);
		onePubOneSubWithTwoMiddleNode(10000);
		
	}
	
}
