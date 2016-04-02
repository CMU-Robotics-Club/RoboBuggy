import com.roboclub.robobuggy.messages.BatteryLevelMessage;
import org.junit.Test;

import static org.junit.Assert.fail;


/**
 * makes sure that we can create a battery level message correctly
 */
public class TestBatteryLevel {

	/**
	 * tries to create a new battery level message
	 */
	@Test
	public void test() {
		BatteryLevelMessage battery = new BatteryLevelMessage(10);
		if(battery.getBatteryLevel() != 10){
			fail("battery level does not match");
		}

	}

}
