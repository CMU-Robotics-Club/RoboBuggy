import java.util.Date;


public class SteeringParser extends BasicParser {

	@Override
	public void parse (){
		new Thread(new Runnable() {
			public void run() {
				while(true) {
					try {
						String toParse = q.take();
						String[] parsed = toParse.split(",");

						//sanity checks
						assert parsed.length == 3;
//						assert parsed[0] == "sensors/steering";
						
						Date date = makeDate(parsed[1]);
						double angle = Double.parseDouble(parsed[2]);
						
						//sleep magic here
						sleep(date);
						
						//publish value here
						System.out.println(String.format("Steering:      Current Date: %s, Date: %s, Steer Angle: %f",
								BasicParser.makeStringFromDate(new Date()), BasicParser.makeStringFromDate(date), angle));						
						
					} catch (InterruptedException e) {
						System.out.println("Interrupted");
					}
				}
			}
		}).start();
	}
}
