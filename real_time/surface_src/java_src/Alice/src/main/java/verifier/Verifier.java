package verifier;
import java.io.*;
import java.util.*;

import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.ros.Subscriber;

//TODO: Write a VerifierChannel (like SensorChannel in BuggyRos), then make an ImuVerifierNode like ImuNode and move this code there.

public class Verifier {
	//Dim global variables:
	static double x, y, z, a=0, b=0, c=0;
	static boolean started = false;
	static int count = 0;
	static double[] thresh = {5, 8, 10}; //Thresholds determined empirically from moving files (roughly 1.5*maxChange)


	//public static void main(String[] args) throws FileNotFoundException {
	public Verifier(){	
		//Potentially real-time code
		Publisher p = new Publisher("IMUstatus");
		Subscriber output = new Subscriber(SensorChannel.IMU.getMsgPath(), new MessageListener(){
			@Override
				public void actionPerformed(String topicName, Message m) {
			ImuMeasurement mydata = (ImuMeasurement) m;
			checkData(mydata, p);
		}
		});
		
		//Testing code; reads from a file
		/*Scanner scanner = new Scanner(new File("/Users/davidneiman/Desktop/CMU/Roboclub/sensorsMoving.txt"));
		while(scanner.hasNext()){
			checkData(scanner.nextLine());
		}
		scanner.close();*/
	}
	
	//For offline code
	public static void checkData(String inp){
		StringTokenizer st = new StringTokenizer(inp, ",");
		st.nextToken();
		String w = st.nextToken();
		String s = st.nextToken();
		if(!s.equals("START")){ //Check for non-data lines
			count++;
			x = Double.parseDouble(s);
			y = Double.parseDouble(st.nextToken());
			z = Double.parseDouble(st.nextToken());
			if(started && (Math.abs(x-a) > thresh[0] || Math.abs(y-b) > thresh[1] || Math.abs(z-c) > thresh[2])){
				System.out.println(count + ". ERROR!");
				System.out.println(w); //Dump the timestamp
				System.out.println((a-x) + " " + (b-y) + " " + (c-z));		
				//Error message
			}
			else{
				//Good message
			}
			a=x;b=y;c=z;
			started = true;
		}
	}
	
	//For real-time code
	public static void checkData(ImuMeasurement inp, Publisher p){
		Date w = inp.timestamp;
		count++;
		x = inp.yaw;
		y = inp.pitch;
		z = inp.roll;
		if(Math.abs(x-a) > thresh[0] || Math.abs(y-b) > thresh[1] || Math.abs(z-c) > thresh[2]){
			System.out.println(count + ". ERROR!");
			System.out.println(w); //Print the timestamp
			System.out.println((a-x) + " " + (b-y) + " " + (c-z));		
			//p.publish(new Message("No problems"));
		}
		else{
			System.out.println("No problems");
			//p.publish(new Message("BAD DATA!!!"));
		}
		a=x;b=y;c=z;
		started = true;
	}
	
}