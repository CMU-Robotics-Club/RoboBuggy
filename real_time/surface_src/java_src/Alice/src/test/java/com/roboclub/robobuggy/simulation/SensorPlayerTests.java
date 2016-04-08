package com.roboclub.robobuggy.simulation;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonElement;
import com.roboclub.robobuggy.messages.BrakeMessage;
import com.roboclub.robobuggy.messages.RobobuggyLogicNotificationMeasurement;

/**
 * Created by vivaanbahl on 2/5/16.
 */
public class SensorPlayerTests {

    /**
     * makes sure we can parse a log file
     * @param args default
     */
    public static void main(String[] args) {

        Gson gson = new GsonBuilder().create();
        String logFile = "{\n" +
                "    \"name\": \"Robobuggy Data Logs\",\n" +
                "    \"schema_version\": 1.0,\n" +
                "    \"date_recorded\": \"2016-02-05 13:59:33.618\",\n" +
                "    \"software_version\": \"1.0.0\",\n" +
                "    \"sensor_data\": [\n" +
                "        {\"VERSION_ID\":\"logic_notification\",\"message\":\"StartingGUI\",\"level\":\"NOTE\"," +
                "\"timestamp\":\"Feb 5, 2016 1:59:24 PM\",\"DATE_FORMAT\":\"yyyy-MM-dd HH:mm:ss.SSS\"},\n" +
                "        {\"VERSION_ID\":\"logic_notification\",\"message\":\"Robobuggy Logic Notfication started\"" +
                ",\"level\":\"NOTE\",\"timestamp\":\"Feb 5, 2016 1:59:31 PM\",\"DATE_FORMAT\":\"yyyy-MM-dd HH:mm:ss." +
                "SSS\"},\n" +
                "        {\"VERSION_ID\":\"logic_notification\",\"message\":\"start/stop button was pressed\"," +
                "\"level\":\"NOTE\",\"timestamp\":\"Feb 5, 2016 1:59:33 PM\",\"DATE_FORMAT\":\"yyyy-MM-dd HH:mm:ss.SSS\"},\n" +
                "        {\"VERSION_ID\":\"gui_logging_buttonV0.0\",\"timestamp\":\"Feb 5, 2016 1:59:33 PM\"" +
                ",\"lm\":\"START\",\"DATE_FORMAT\":\"yyyy-MM-dd HH:mm:ss.SSS\"},\n" +
                "        {\"VERSION_ID\":\"logic_notification\",\"message\":\"Starting up logging thread!\"" +
                ",\"level\":\"NOTE\",\"timestamp\":\"Feb 5, 2016 1:59:33 PM\",\"DATE_FORMAT\":\"yyyy-MM-dd HH:mm:ss.SSS\"},\n" +
                "        {\"VERSION_ID\":\"logic_notification\",\"message\":\"start/stop button was pressed\",\"level\":" +
                "\"NOTE\",\"timestamp\":\"Feb 5, 2016 1:59:34 PM\",\"DATE_FORMAT\":\"yyyy-MM-dd HH:mm:ss.SSS\"},\n" +
                "        {\"VERSION_ID\":\"STOP\"}\n" +
                "    ],\n" +
                "    \"data_breakdown\" : {\"logging_button\":1,\"gps\":0,\"imu\":0,\"encoder\":0,\"brake\"" +
                ":0,\"steering\":0,\"fp_hash\":0,\"logic_notification\":5,\"pose\":0,\"reset\":0,\"state\":0}\n" +
                "}\n";
        String logLine = "        {\"VERSION_ID\":\"logic_notification\",\"message\":\"StartingGUI\",\"level\"" +
                ":\"NOTE\",\"timestamp\":\"Feb 5, 2016 1:59:24 PM\",\"DATE_FORMAT\":\"yyyy-MM-dd HH:mm:ss.SSS\"}";

        RobobuggyLogicNotificationMeasurement m = gson.fromJson(logLine, RobobuggyLogicNotificationMeasurement.class);
        JsonElement logFileJson = gson.fromJson(logFile, JsonElement.class);
        System.out.println(logFileJson.getAsString());

        System.out.printf(String.valueOf((BrakeMessage) gson.fromJson(logLine, BrakeMessage.class)));

        System.out.println(m.toString());


    }

}
