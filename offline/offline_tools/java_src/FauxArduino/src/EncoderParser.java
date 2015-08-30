import java.util.Date;


public class EncoderParser extends BasicParser {
	
	@Override
	public void parse () {
		new Thread(new Runnable() {
			public void run() {
				while(true) {
					try {
						String toParse = q.take();
						String[] parsed = toParse.split(",");

						//sanity checks
						assert parsed.length == 6;
//						assert parsed[0] == "sensors/encoder";
						
						Date date = makeDate(parsed[1]);
						double magic1 = Double.parseDouble(parsed[2]);
						double distance = Double.parseDouble(parsed[3]);
						double velocity = Double.parseDouble(parsed[4]);
						double accel = Double.parseDouble(parsed[5]);
						
						
						//sleep magic here
						sleep(date);
						
						//publish value here
						System.out.println(String.format("Encoder:       Current Date: %s, Date: %s, Magic1: %f, Distance: %f,"
								+ "Velocity: %f, Accel: %f", BasicParser.makeStringFromDate(new Date()), BasicParser.makeStringFromDate(date), 
								magic1, distance, velocity, accel));						
						
					} catch (InterruptedException e) {
						System.out.println("Interrupted");
					}
				}
			}
		}).start();
	}
}
