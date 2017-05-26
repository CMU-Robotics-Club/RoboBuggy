/**
 * Created by Matt on 8/29/2014.
 */
// It looks like we have to use RXTX for java/arduino serial communication.
// Question I don't have answers to yet: how to choose which serial port is data, logging.
//   Prolly could just have a preliminary byte sent, and that would distinguish.
// TODO figure out how to do a local install of the rxtx binaries vs. global install

// TODO Note that, when you do a global install you WILL need to invalidate the cache of your IDE
package com.roboclub.robobuggy;

import gnu.io.*;
import java.util.*;
import java.io.InputStream;
import java.io.OutputStream;
public class main {
    
	public static void main(String args[]) {

        try {
             CommPortIdentifier portIdentifier = CommPortIdentifier.getPortIdentifier("COM5");
            
             CommPort commPort = portIdentifier.open("owner-name", 2000);
            SerialPort serialPort = null;
            int rate = 9600;
            int databits = SerialPort.DATABITS_8;
            int stopbits = SerialPort.STOPBITS_1;
            int parity = SerialPort.PARITY_NONE;
            if (commPort instanceof SerialPort) {
                serialPort = (SerialPort) commPort;
                serialPort.setSerialPortParams(rate, databits, stopbits, parity);
            }
            InputStream in = serialPort.getInputStream();
            OutputStream out = serialPort.getOutputStream();
            
            byte[] arr = new byte[10];
            
            while(true) {
            	if(in.available() > 10)
            	{
            		int n = in.read(arr);
            		String temp = new String(arr);
            		System.out.println(temp);
            		//System.out.println((char)arr[0]);
            		System.out.println(Arrays.toString(arr));
            	}
            }
            //
            // Use port identifier for acquiring the port
        } catch (Exception e) {
            System.out.print("failed");
        }
    }
}