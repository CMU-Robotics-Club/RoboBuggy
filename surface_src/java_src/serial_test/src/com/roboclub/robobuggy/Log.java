package com.roboclub.robobuggy;

public interface Log {

	// Logs a message 
	// This is meant to log messages sent from the code, for debugging purposes 
	// 
	// In order to log DATA (e.g. measurements) please use data log.
	// 
	// Inspired by the logcat interface, on android. 
	
	// Levels are (least important to most important):
	//  (v)erbose (least important)
	//  (d)ebug 
	//  (i)nformation
	//  (w)arning
	//  (e)rror
	
	public void v(String tag, String message);
	
	public void d(String tag, String message);
	
	public void i(String tag, String message);
	
	public void w(String tag, String message);
	
	public void e(String tag, String message);
}
