import java.util.Date;

public class LoggingStartParser extends BasicParser {

	@Override
	public void parse () {
		new Thread(new Runnable() {
			public void run() {
				while(true) {
					try {
						String toParse = q.take();
						String[] parsed = toParse.split(",");

						//sanity checks
						assert parsed.length == 11;
//						assert parsed[0] == "sensors/gps";
						
						Date date = makeDate(parsed[1]);
						String type = parsed[2];
												
						//sleep magic here
						sleep(date);
						
						//publish value here
						System.out.println(String.format("LoggingButton: Current Date: %s, Date: %s, Case: %s",
								BasicParser.makeStringFromDate(new Date()), BasicParser.makeStringFromDate(date), type));
						
					} catch (InterruptedException e) {
						System.out.println("Interrupted");
					}
				}
			}
		}).start();
	}
}
