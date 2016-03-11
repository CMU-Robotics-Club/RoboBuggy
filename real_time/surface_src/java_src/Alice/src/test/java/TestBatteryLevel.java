import static org.junit.Assert.*;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import com.roboclub.robobuggy.messages.BatteryLevelMessage;


public class TestBatteryLevel {

	@Before
	public void setUp() throws Exception {
	}

	@After
	public void tearDown() throws Exception {
	}

	@Test
	public void test() {
		BatteryLevelMessage battery = new BatteryLevelMessage(10);
		if(battery.getBatteryLevel() != 10){
			fail("battery level does not match");
		}

	}

}
