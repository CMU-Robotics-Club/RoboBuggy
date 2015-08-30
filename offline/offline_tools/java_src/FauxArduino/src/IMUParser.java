import java.util.Date;

public class IMUParser extends BasicParser {
			
	@Override
	public void parse() {
		new Thread(new Runnable() {
			public void run() {
				while(true) {
					try {
						String toParse = q.take();
						String[] parsed = toParse.split(",");

						//sanity checks
						assert parsed.length == 5;
//						assert parsed[0] == "sensors/imu";
						
						Date date = makeDate(parsed[1]);
						double aX = Double.parseDouble(parsed[2]);
						double aY = Double.parseDouble(parsed[3]);
						double aZ = Double.parseDouble(parsed[4]);
//						double rX = Double.parseDouble(parsed[5]);
//						double rY = Double.parseDouble(parsed[6]);
//						double rZ = Double.parseDouble(parsed[7]);
//						double mX = Double.parseDouble(parsed[8]);
//						double mY = Double.parseDouble(parsed[9]);
//						double mZ = Double.parseDouble(parsed[10]);
						
						//sleep magic here
						sleep(date);
						
						//publish value here
						System.out.println(String.format("IMU:           Current Date: %s, Date: %s, aX: %f, aY: %f, aZ: %f,"
								/*+ "rX: %f, rY: %f, rZ: %f, mX: %f, mY: %f, mZ: %f"*/, BasicParser.makeStringFromDate(new Date()), BasicParser.makeStringFromDate(date),
								aX, aY, aZ /*,rX, rY, rZ, mX, mY, mZ*/));						
						
					} catch (InterruptedException e) {
						System.out.println("Interrupted");
					}
				}
			}
		}).start();
	}
}