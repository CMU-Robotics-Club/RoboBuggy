package com.roboclub.robobuggy.logging;
import java.util.*;
import java.io.*;
public class OldLogConverter {

	public static void main(String[] args) throws IOException {
		Scanner scanner = new Scanner(new File("/Users/davidneiman/Desktop/sensors.txt"));
		PrintStream writer = new PrintStream(new File("/Users/davidneiman/Desktop/sensorsJSON.txt"));
		ArrayList<String> sensor_type = new ArrayList();
		ArrayList<Integer> sensor_quant = new ArrayList();
		
		//Read in the first set of data so we get the date we're recording on
		String s = scanner.nextLine();
		StringTokenizer st = new StringTokenizer(s, ",");
		String type = st.nextToken().substring(8); //Could be IMU, GPS, encoder, steering
		StringTokenizer st2 = new StringTokenizer(st.nextToken(), " ");
		//The next token is the date, but in yyyy-mm-dd format; I want mm/dd/yyyy format
		StringTokenizer stdate = new StringTokenizer(st2.nextToken(), "-");
		String[] dateymd = {stdate.nextToken(), stdate.nextToken(), stdate.nextToken()};
		String date_recorded = dateymd[1] + "/" + dateymd[2] + "/" + dateymd[0]; //This should now be in mm/dd/yyyy format
		String timestamp = st2.nextToken();
		
		//Add heading
		String logname = "Robobuggy Data Logs";
		String schema_version = "1.0";
		String software_version = "1.0.0";
		writer.println("{");
		writer.println("\t\"name: \"" + logname + "\",");
		writer.println("\t\"schema_version\":" + schema_version + ",");
		writer.println("\t\"date_recorded\": \"" + date_recorded + "\",");
		writer.println("\t\"software_version\":" + software_version + ",");
		writer.println("\t\"sensor_data\":");
		writer.println("\t[");
		
		//Write the first set of data we read in before
		writeSensorData(scanner, writer, st, type, timestamp);
		
		//Main loop to log sensor data
		while(scanner.hasNextLine()){
			s = scanner.nextLine();
			st = new StringTokenizer(s, ",");
			type = st.nextToken().substring(8); //Could be IMU, GPS, encoder, steering
			st2 = new StringTokenizer(st.nextToken(), " ");
			st2.nextToken(); //Skip the date
			timestamp = st2.nextToken();
			writeSensorData(scanner, writer, st, type, timestamp);
			updateDataBreakdown(sensor_type, sensor_quant, type);
		}
		//End sensor data part
		writer.println("\t],");
		
		//Print the breakdown
		writer.println("\t\"data_breakdown\": {");
		for(int x = 0; x < sensor_type.size(); x++){
			writer.println("\t\t\"" + sensor_type.get(x) + "\": " + sensor_quant.get(x) +",");
		}
		writer.println("\t}");
		
		//Clean up and end
		writer.print("}");
		System.out.println("Done converting");
	}
	
	//Update the types of sensors we have readings for as well as the number of readings
	public static void updateDataBreakdown(ArrayList<String> sensor_type, ArrayList<Integer> sensor_quant, String type){
		//If you don't have the sensor, add it to the list and note that you now have 1 reading for it
		if(sensor_type.indexOf(type) == -1){
			sensor_type.add(type);
			sensor_quant.add(1);
		}
		//Otherwise, increment the number of readings you have
		else{
			sensor_quant.set(sensor_type.indexOf(type), sensor_quant.get(sensor_type.indexOf(type)).intValue()+1);
		}
	}
	
	//Given the necessary data, write it to a file in the new format. Lots of brute-force and case statements here
	public static void writeSensorData(Scanner scanner, PrintStream writer, StringTokenizer st, String type, String timestamp){
		writer.println("\t\t{");
		writer.println("\t\t\t\"timestamp\": \"" + timestamp + "\",");
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
		writer.print("\t\t}");
		if(scanner.hasNextLine()){
			writer.println(",");
		}
		else{
			writer.println();
		}
	}
}
