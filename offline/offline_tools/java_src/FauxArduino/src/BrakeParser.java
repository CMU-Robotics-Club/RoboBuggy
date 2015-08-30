import java.util.Date;

public class BrakeParser extends BasicParser {

	@Override
	public void parse () {
		new Thread(new Runnable() {
			public void run() {
				while(true) {
					try {
						String toParse = q.take();
						String[] parsed = toParse.split(",");

						//sanity checks
						assert parsed.length == 4;
//						assert parsed[0] == "sensors/gps";
						
						Date date = makeDate(parsed[1]);
						String type = parsed[2];
						boolean val = Boolean.parseBoolean(parsed[3]);
												
						//sleep magic here
						sleep(date);
						
						//publish value here
						System.out.println(String.format("Brake          Current Date: %s, Date: %s, Case: %s, Val: %b",
								BasicParser.makeStringFromDate(new Date()), BasicParser.makeStringFromDate(date), type, val));
						
					} catch (InterruptedException e) {
						System.out.println("Interrupted");
					}
				}
			}
		}).start();
	}
}
