package com.roboclub.robobuggy.logging;
import java.util.*;
import java.io.*;
public class OldLogConverter {

	public static void main(String[] args) throws IOException {
		// TODO Auto-generated method stub
		Scanner scanner = new Scanner(new File("/Users/davidneiman/Desktop/sensors.txt"));
		PrintStream writer = new PrintStream(new File("/Users/davidneiman/Desktop/sensorsJSON.txt"));
		//Add heading
		
		while(scanner.hasNextLine()){
			writer.println("\t\t{");
			String s = scanner.nextLine();
			StringTokenizer st = new StringTokenizer(s, ",");
			String type = st.nextToken().substring(8); //Could be IMU, GPS, encoder, steering
			writer.println("\t\t\t\"timestamp\": \"" + st.nextToken() + "\",");
			writer.println("\t\t\t\"data\": {");
			writer.println("\t\t\t\t\"name\": \"" + type + "\"");
			switch(type){
			case "steering":
				writer.println("\t\t\t\t\"value\": \"" + st.nextToken() + "\",");
				break;
			case "encoder":
				writer.println("\t\t\t\t\"params\": {");
				writer.println("\t\t\t\t\t\"val1\": \"" + st.nextToken() + "\",");
				writer.println("\t\t\t\t\t\"val2\": \"" + st.nextToken() + "\",");
				writer.println("\t\t\t\t\t\"val3\": \"" + st.nextToken() + "\",");
				writer.println("\t\t\t\t\t\"val4\": \"" + st.nextToken() + "\",");
				writer.println("\t\t\t\t},");
				break;
			case "logging_button":
				writer.println("\t\t\t\t\"button\": \"" + st.nextToken() + "\",");
				break;
			case "gps":
				writer.println("\t\t\t\t\"params\": \"" + st.nextToken() + "\",");
				writer.println("\t\t\t\t\t\"location\": {");
				writer.println("\t\t\t\t\t\"lat\": \"" + st.nextToken() + "\",");
				st.nextToken(); //Direction
				writer.println("\t\t\t\t\t\"lon\": \"" + st.nextToken() + "\",");
				st.nextToken(); //Direction
				writer.println("\t\t\t\t\t\"val1\": \"" + st.nextToken() + "\",");
				writer.println("\t\t\t\t\t\"val2\": \"" + st.nextToken() + "\",");
				writer.println("\t\t\t\t\t\"val3\": \"" + st.nextToken() + "\",");
				writer.println("\t\t\t\t\t\"val4\": \"" + st.nextToken() + "\",");
				writer.println("\t\t\t\t},");
				break;
			case "imu":
				writer.println("\t\t\t\t\"params\": {");
				//Note: I'm not sure what order these read in in.
				writer.println("\t\t\t\t\t\"yaw\": \"" + st.nextToken() + "\",");
				writer.println("\t\t\t\t\t\"pitch\": \"" + st.nextToken() + "\",");
				writer.println("\t\t\t\t\t\"roll\": \"" + st.nextToken() + "\",");
				writer.println("\t\t\t\t},");
				break;
			default: 
				System.out.println("Unknown sensor: " + type);
			}
			writer.println("\t\t\t}");
			writer.println("\t\t},");
		}
		System.out.println("Done converting");
	}

}
