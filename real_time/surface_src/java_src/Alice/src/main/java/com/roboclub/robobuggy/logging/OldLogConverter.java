package com.roboclub.robobuggy.logging;
import java.util.*;
import java.io.*;
public class OldLogConverter {
	
static ArrayList<String> sensor_type = new ArrayList<String>();
static ArrayList<Integer> sensor_quant = new ArrayList<Integer>();

	public static void main(String[] args) throws IOException {
		boolean fileinput = true;
		Scanner scanner; PrintStream writer;
		
		if(fileinput){
			scanner = new Scanner(System.in);
			System.out.println("Input filepath to old data file.");
			String inputpath = scanner.nextLine();
			System.out.println("Input filepath to new data file.");
			String outputpath = scanner.nextLine();
			scanner.close();
			scanner = new Scanner(new File(inputpath));
			writer = new PrintStream(new File(outputpath));
		}
		else{
			scanner = new Scanner(new File("/Users/davidneiman/Desktop/sensors.txt"));
			writer = new PrintStream(new File("/Users/davidneiman/Desktop/sensorsJSON.txt"));
		}
		
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
		writer.println("    \"name\": \"" + logname + "\",");
		writer.println("    \"schema_version\": " + schema_version + ",");
		writer.println("    \"date_recorded\": \"" + date_recorded + "\",");
		writer.println("    \"software_version\": \"" + software_version + "\",");
		writer.println("    \"sensor_data\": [");
		
		//Write the first set of data we read in before
		writeSensorData(scanner, writer, st, type, timestamp);
		
		//Main loop to log sensor data
		while(scanner.hasNextLine()){
			s = scanner.nextLine();
			st = new StringTokenizer(s, ",");
			type = st.nextToken().substring(8); //Could be IMU, GPS, encoder, steering
			st2 = new StringTokenizer(st.nextToken(), " ");
			timestamp = st2.nextToken() + " " + st2.nextToken(); //Date time
			writeSensorData(scanner, writer, st, type, timestamp);
		}
		//End sensor data part
		writer.println("    ],");
		
		//Print the breakdown
		writer.print("    \"data_breakdown\" : {");
		for(int x = 0; x < sensor_type.size(); x++){
			writer.print("\"" + sensor_type.get(x) + "\":" + sensor_quant.get(x));
			//Omit last comma
			if(x < sensor_type.size()-1){
				writer.print(",");
			}
		}
		writer.println("}");
		
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
	//Note: Anything that would be a tab is a quadruple space
	//Another note: Capitalization of sensor names is off
	public static void writeSensorData(Scanner scanner, PrintStream writer, StringTokenizer st, String name, String timestamp){
		if(name.equalsIgnoreCase("GPS")){name = "GPS";}
		if(name.equalsIgnoreCase("IMU")){name = "IMU";}
		if(name.equals("logging_button")){name = "logging button";}
		else{name = name.substring(0, 1).toUpperCase() + name.substring(1);}
		writer.print("        {\"name\":\"" + name + "\",");
		switch(name){
		case "Steering":
			writer.print("\"params\":{\"angle\": \"" + st.nextToken() + "\"},");
			break;
		case "Encoder":
			//dataword, d, v, a
			String dataword = st.nextToken();
			String d = st.nextToken();
			String v = st.nextToken();
			String a = st.nextToken();
			if(dataword.equals("NaN")){
				dataword = "null";
			}
			if(d.equals("NaN")){
				d = "null";
			}
			if(v.equals("NaN")){
				v = "null";
			}
			if(a.equals("NaN")){
				a = "null";
			}
			writer.print("\"params\":{\"dataword\":" + dataword + ",\"distance\":" + d + ",\"velocity\":" + v + ",\"acceleration\":" + a + "},");
			break;
		case "logging button":
			writer.print("\"params\":{\"logging_status\":\"" + st.nextToken().toLowerCase() + "\"},");
			break;
		case "GPS":
			st.nextToken(); //GPS data stores a second time; I'm not really sure what it is, but we don't need it
			//Latitude/longitude
			writer.print("\"params\":{\"latitude\":" + st.nextToken() + ",\"lat_direction\":\"" + st.nextToken() + "\",\"longitude\":" + st.nextToken() + ",\"long_direction\":\"" + st.nextToken() + "\",");
			//Other parameters
			writer.print("\"gps_quality\":\"" + st.nextToken() + "\",\"num_satellites\":\"" + st.nextToken() + "\",\"HDOP\":" + st.nextToken() + ",\"antenna_altitude\":" + st.nextToken() + "},");
			break;
		case "IMU":
			//Note: I'm not sure what order these read in in. I'm taking an educated guess that the -120 reading is yaw and that therefore the old data is YPR
			String yaw = st.nextToken();
			String pitch = st.nextToken();
			String roll = st.nextToken();
			writer.print("\"params\":{\"roll\":" + roll + ",\"pitch\":" + pitch + ",\"yaw\":" + yaw + "},");
			break;
		default:
			//Note: This doesn't print any parameters, but you'll still have the sensor name and timestep in the log
			System.out.println("Unknown sensor: " + name);
		}
		//Appending the timestamp to the end of everything is universal 
		writer.print("\"timestamp\":\"" + timestamp + "\"}");
		if(scanner.hasNextLine()){
			writer.println(",");
		}
		else{
			writer.println();
		}
		updateDataBreakdown(sensor_type, sensor_quant, name);
	}
}
