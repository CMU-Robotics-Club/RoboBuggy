import java.util.Date;

public class GPSParser extends BasicParser {

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
						Date otherDate = makeDate(parsed[2]);
						double lat = Double.parseDouble(parsed[3]);
						String latDir = parsed[4];
						double longi = Double.parseDouble(parsed[5]);
						String longiDir = parsed[6];
						double magic1 = Double.parseDouble(parsed[7]);
						double magic2 = Double.parseDouble(parsed[8]);
						double magic3 = Double.parseDouble(parsed[9]);
						double magic4 = Double.parseDouble(parsed[10]);
												
						//sleep magic here
						sleep(date);
						
						//publish value here
						System.out.println(String.format("GPS:           Current Date: %s, Date: %s, Other Date: %s, Lat: %s,"
								+ " %s Long: %s %s %f %f %f %f", BasicParser.makeStringFromDate(new Date()), BasicParser.makeStringFromDate(date),
								BasicParser.makeStringFromDate(otherDate), lat, latDir, longi, longiDir, magic1, magic2, magic3, magic4));
						
					} catch (InterruptedException e) {
						System.out.println("Interrupted");
					}
				}
			}
		}).start();
	}
}
