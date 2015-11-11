package com.roboclub.robobuggy.logging.autoLogging;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Hashtable;

import org.apache.commons.logging.Log;

import com.orsoncharts.util.json.JSONObject;
import com.google.api.services.drive.Drive.Files;
import com.google.api.services.drive.model.ChildList;
import com.google.api.services.drive.model.ParentReference;
import com.roboclub.robobuggy.logging.LogDataType;
import com.roboclub.robobuggy.main.MESSAGE_LEVEL;
import com.roboclub.robobuggy.main.RobobuggyLogicException;
import com.roboclub.robobuggy.main.config;

//TODO things to add in future versions: 
// When a user edits a file outside of this system it should be detected and updated on the server
// When a file is deleted on the server or locally it should be propagated to the server
// add logic for automatically removing logs that have not been used recently from local storage 

/**
 * 
 * The AutoLogging class manages the storage and availability of log files and log data
 * The system keeps track of when this computer last accessed each log file, and how often 
 * that file is used.
 * 
 * A heap is maintained of what log files to cache.  This way the total space used by log files does 
 * not take up too much space on a users  computer.  
 * 
 * 
 * When a new log file is created or a log file is changed then it is updated on the server
 * 
 * Known Problems: 
 *      A race condition exists when two computers attempt to upload data about a log change at the same time. 
 * 
 * Behaviors: 
 * 		B1: All log files that have not yet been uploaded to the server shall be uploaded to the server
 * 		B2: The system should be able to download log files from the server
 * 		B3: The system should have a list of all log files and statistics about what happened in that log file
 * 
 * @author Trevor Decker
 *
 */
public class autoLogging {	
	private Hashtable<String, LogDataType> logData;
	private Thread logThread = null;

	//on google drive every file and folder is given a unique identifying string 
	private String DriveStorageFolder_id;
	private File localFolderPath;
	private static autoLogging autoLogger = null;
	
	public static autoLogging startAutoLogger(File whereToSave,String DriveStorageFolder_id){
		if(autoLogger== null){
			try {
				autoLogger = new autoLogging( whereToSave, DriveStorageFolder_id);
			} catch (IOException e) {
				new RobobuggyLogicException("error while creating autologger", MESSAGE_LEVEL.EXCEPTION);
			}
		}else{
			new RobobuggyLogicException("autoLogger already created", MESSAGE_LEVEL.EXCEPTION);
		}
		return autoLogger;
	}
	
	public static autoLogging getLogger(){
		if(autoLogger==null){
			 new RobobuggyLogicException("you need to start the auto logger before using it", MESSAGE_LEVEL.EXCEPTION);
		}
		return autoLogger;
	}
	
	/**
	 *  The constructor for the auto logger, will load any local files 
	 * @param whereToSave
	 * @param DriveStorageFolder_id
	 * @throws IOException
	 */
	private autoLogging(File whereToSave,String DriveStorageFolder_id) throws IOException{
		logData = new Hashtable<String, LogDataType>();
		setWhereToSave(whereToSave);
		setServerFolderId(DriveStorageFolder_id);
		readLogData();
	}

	public void startLogSync(){
		//can only run one sync at a time so we should stop the old one 
		if(logThread!= null && logThread.isAlive()){
			stopLogSync();
		}
		logSyncThread newThead = new logSyncThread(getWhereToSave());
        logThread = new Thread(newThead);
        logThread.start();		
	}
	
	//will cause for the thread that is currently syncing with the server to stop
	public void stopLogSync(){
		//TODO make this cleaner 
		if(logThread != null){
			logThread.stop();
		}
	}
	
	public class logSyncThread implements Runnable {
		File whereToSave;

	    public logSyncThread(File whereToSave) {
	    	this.whereToSave = whereToSave;
	    }

	    public void run() {
	    	try {
				uploadLogs();
				downloadLogData(DriveStorageFolder_id, whereToSave);
			} catch (IOException e) {
				new RobobuggyLogicException("trouble syncing", MESSAGE_LEVEL.WARNING);
			}	    }
	}
	
	
	/**
	 * evaluates to the number of logs that are currently being tracked 
	 * @return
	 */
	public int getNumLogs(){
		return logData.size();
	}
	
	/**
	 * 
	 * @param newServerFolderId
	 */
	public void setServerFolderId(String newServerFolderId){
		DriveStorageFolder_id = newServerFolderId;
	}
	
	/**
	 * 
	 * @return
	 */
	public String getServerFolderId(){
		return DriveStorageFolder_id;
	}
	
	/**
	 * should be called when the program is finished to save to server and save to local file 
	 * @return
	 * @throws IOException
	 */
	public boolean save() throws IOException{
		boolean localSaveState = saveLogDataToFolders();
		boolean serverSaveState = saveLogDataToServer();
		return localSaveState && serverSaveState;
	}
	
	/**
	 * returns a file reference of the local path of where log files are to be saved
	 * @return
	 */
	public File getWhereToSave(){
		return localFolderPath;
	}
	
	/**
	 * 
	 * @param whereToSave
	 */
	public void setWhereToSave(File whereToSave){
		//makes sure that the folder exists, if it does not create the folder
		if(!whereToSave.isDirectory()){	
			whereToSave.mkdir();
		}
		
		this.localFolderPath = whereToSave;
	}
	
	
	/**
	 * Evaluates to a list of all of the log files
	 * @return
	 */
	ArrayList<LogDataType> getListOfLogFiles(){
		ArrayList<LogDataType> result = new ArrayList<LogDataType>();
		for(LogDataType l: logData.values()){
			result.add(l);
		}
		return result;
	}	
	
	/**
	 * will overwrite the current log file, should call readLogData before calling saveLogDataToFolder
	 * @return
	 * @throws IOException
	 */
	public boolean saveLogDataToFolders() throws IOException{
		boolean allLogsSavedCorrectly = true;
		ArrayList<LogDataType> data = getListOfLogFiles();
		for (LogDataType thisLogData : data) {
			boolean thisLogSavedState = thisLogData.saveThisLogDataToFolder();
			if(!thisLogSavedState){
				allLogsSavedCorrectly = false;
			}
		}		
		return allLogsSavedCorrectly;
	}
	
	/**
	 * Assumes that any log files have already been loaded, ie can override any files on disk
	 * @param logFolder
	 * @returnsaveLogDataLocally
	 * @throws IOException
	 */
	boolean saveLogDataLocally(LogDataType logData) throws IOException{
		File logFolder = logData.getLogRefrenceOnComputer();
		if(!logFolder.exists() ){
			logFolder.mkdirs();
		}
		if(!logFolder.isDirectory()){
			new RobobuggyLogicException("Attempted to save a log folder that is not a folder", MESSAGE_LEVEL.EXCEPTION);
		}
		
		//if a file is already at location then it will be over written 
		File logDataFile = new File(logFolder.getPath()+"/"+LogDataType.FILE_NAME);
		logDataFile.delete();
		logData.saveThisLogDataToFolder();
		return true;
	}
	
	/**
	 * 
	 * @param logFolder
	 * @return
	 * @throws FileNotFoundException 
	 */
	LogDataType readALogData(File logFolder) throws FileNotFoundException{
		LogDataType newLogData = LogDataType.readThisLogDataFromFolder(logFolder);
		String oldLogData_key = lookUpLog(newLogData);
		if(oldLogData_key == null){
			//this log is not already being tracked so we should start tracking it 
			logData.put(newLogData.getKey(), newLogData);
		}else{
			//this log is not new so we should update the log
			LogDataType mergedLogData = LogDataType.merge(logData.get(oldLogData_key), newLogData);
			logData.replace(oldLogData_key,mergedLogData);
			newLogData = mergedLogData;
		}
		return newLogData;
	}
	
	/**
	 * reads every folder inside whereToSave if it is a logfile then add it to the logDataType structure 
	 * @return
	 * @throws FileNotFoundException 
	 */
	boolean readLogData() throws FileNotFoundException{
		boolean result = true;
		for(File f : localFolderPath.listFiles()){
			if(LogDataType.isLog(f)){
				if(readALogData(f)== null){
					result = false;
				}
			}
		}
		return result;
	}

	/**
	 * adds logging data to a log folder that may or may not exist. 
	 * @param LogLocation
	 * @return
	 * @throws IOException
	 */
	public boolean startTrackingLog(File newLogLocation) throws IOException{
		System.out.println("logfile:"+newLogLocation);
		LogDataType newLog = new LogDataType(newLogLocation);
		LogDataType oldLog = logData.get(newLog.getKey());
		if(oldLog == null){
			//is a new log
			logData.put(newLog.getKey(), newLog);
			saveLogDataLocally(newLog);
		}else{
			// is not a new log so we should merge 
			newLog = oldLog.merge(oldLog, newLog);
			logData.replace(newLog.getKey(), newLog);
		}
		return true;//uploadLog(newLog);
	}
	
	/**
	 * assumes that data on the server has been saved. This can cause race conditions
	 * @return
	 * @throws IOException
	 */
	public boolean saveLogDataToServer() throws IOException{
		boolean allLogsSavedCorrectly = true;
		ArrayList<LogDataType> data = getListOfLogFiles();
		for (LogDataType thisLogData : data) {
			if(!thisLogData.isUpToDateOnServer()){
				if(!uploadLog(thisLogData)){
					allLogsSavedCorrectly = false;
				}
			}
		}		
		return allLogsSavedCorrectly;
	}
	
	/**
	 * assumes that most recent changes have been done locally, this can casue raceconditions and what is on the server will be over wirtten 
	 * @param thisLog
	 * @return
	 * @throws IOException
	 */
	boolean uploadLog(LogDataType thisLog) throws IOException{
		if(config.IN_OFFLINE_MODE){
			return false;
		}
		
		//check to see if log has already been uploaded, if it has been do nothing 
		if(thisLog.isUpToDateOnServer()){
			return true;
		}
		
		//if the log file has not been uploaded then uploaded it	
		ServerCommunication server = ServerCommunication.getInstance();
		if(!thisLog.exsitsOnServer()){
			//we need to create the folder to store this file
			String location = server.createFolder(thisLog.getFolderName(),DriveStorageFolder_id);
			thisLog.setLogRefrenceOnServer(location);
		}
		//we need to upload the Data inside the log so we delete the old data and upload new data 
		server.removeContentOfFolder(thisLog.getLogRefrenceOnServer());
		//now we upload the new data
		ChildList serverFiles = server.getFilesInFolder(thisLog.getLogRefrenceOnServer());
		//makes sure the server reference is valid
		if(thisLog.getLogRefrenceOnServer().equals("")){
			new RobobuggyLogicException("trying to upload a folder to empty location on google drive", MESSAGE_LEVEL.EXCEPTION);
			return false;
		} 
		
		//makes sure that the local folder reference is valid
		if(thisLog.getLogRefrenceOnComputer().equals("")){
			new RobobuggyLogicException("trying to upload a folder with out a path", MESSAGE_LEVEL.EXCEPTION);
			return false;
		}
		boolean result = server.uploadFolder(thisLog.getLogRefrenceOnComputer(),thisLog.getLogRefrenceOnServer());
		if(result){
			thisLog.setUpToDateOnServer(true);
		}else{
			new RobobuggyLogicException("trouble uploading log file, will try again later", MESSAGE_LEVEL.WARNING);
		}
		thisLog.setUpToDateOnServer(true);

		//now we need to update the internal reference in the data structure
		logData.replace(thisLog.getKey(), thisLog);
	    //log saved correctly so we should update our log of it locally 
		saveLogDataLocally(thisLog);
		//now reupload the logDataFile
		server.uploadFile(thisLog.getLocalLogDataFile(), thisLog.getLogRefrenceOnServer());
		
		return true;
	}
	
	/**
	 * 
	 * @param thisLog
	 * @return
	 * @throws IOException
	 */
	boolean downloadLog(LogDataType thisLog) throws IOException{
		if(config.IN_OFFLINE_MODE){
			return false;
		}
		
		ServerCommunication server = ServerCommunication.getInstance();
		server.downloadFolder(thisLog.getLogRefrenceOnComputer(),thisLog.getLogRefrenceOnServer());
		thisLog.setUpToDateOnServer(true);
		return true;
	}
	
	/**
	 * evaluates to null if the given logData is not already being stored
	 * evaluates to a reference to that logData if an equivalent log is being stored 
	 * @param aLogData
	 * @return	
	 */
	public String lookUpLog(LogDataType aLogData){
		//in case the log has not been added to server yet 
		if(aLogData.getKey() == null || aLogData.getKey().equals("")){
			return null;
		}
	
		//makes sure that the key is actually inside the log has table
		if(logData.get(aLogData.getKey()) == null){
			return null;
		}
		return aLogData.getKey();	
	}
	
	/**
	 * goes to every folder inside the server folder and downloads the meta data for those logs  
	 * @param Folder_id
	 * @param whereToSave
	 * @return
	 * @throws IOException
	 */
	boolean downloadLogData(String Folder_id,File whereToSave) throws IOException{
		if(config.IN_OFFLINE_MODE){
			return false;
		}
		
		ServerCommunication server = ServerCommunication.getInstance();
		ArrayList<com.google.api.services.drive.model.File> folders = server.ListFolderMetaDataInDrive(Folder_id);
		for(com.google.api.services.drive.model.File f : folders){
			if(isLogFolderOnDrive(f)){
				System.out.println("downloading"+f.getTitle());

				//for each folder see if we already have its log data, if so merge data
				//if not then download and add the log data 
				LogDataType newLogData = readThisLogDataFromServerFolder(f, whereToSave);
				if(newLogData == null){
					new RobobuggyLogicException("Tried to download a null file", MESSAGE_LEVEL.EXCEPTION);
				}else{
					newLogData.setUpToDateOnServer(true);
					String key = lookUpLog(newLogData);
					if(key != null){
						LogDataType oldData = logData.get(key);
						logData.replace(key, LogDataType.merge(oldData, newLogData));
					}else{
						logData.put(key, newLogData);
					}
				}
			}else{
				System.out.println("is not a log folder"+f.getTitle());
				//if it is a folder then crate that folder locally and recurse
				String newFolderString = whereToSave+"/"+f.getTitle();
				File newFolder = new File(newFolderString);
				newFolder.mkdir();
				downloadLogData(f.getId(),newFolder);
			}
		}
		return false;
	}

	/**
	 * 
	 * @param f
	 * @param parentFile
	 * @return
	 * @throws IOException
	 */
	private LogDataType readThisLogDataFromServerFolder(
			com.google.api.services.drive.model.File f,File parentFile) throws IOException {
		if(config.IN_OFFLINE_MODE){
			//throw an error for trying to connect to the server while in offline mode 
		}
		
		ServerCommunication server =  ServerCommunication.getInstance();
		 ArrayList<com.google.api.services.drive.model.File> files =  server.ListFileMetaDataInDrive(f.getId());
		 for(com.google.api.services.drive.model.File thisFile : files){
			 if(thisFile.getTitle() == LogDataType.FILE_NAME){
				 //download the file 
				 server.downloadFile(thisFile,parentFile);
				 File logFolder = new File(parentFile.getPath()+"/"+thisFile.getTitle());
				 return LogDataType.readThisLogDataFromFolder(logFolder);
			 }
		 }
		return null;
	}

	/**
	 * 
	 * @param f a local machine file path
	 * @return true if f is a folder that encodes a log 
	 * @throws IOException
	 */
	private boolean isLogFolderOnDrive(com.google.api.services.drive.model.File f) throws IOException {
		if(config.IN_OFFLINE_MODE){
			new RobobuggyLogicException("tried to check if a file is a log folder when in offline mode", MESSAGE_LEVEL.EXCEPTION);
			return false;
		}
		ServerCommunication server =  ServerCommunication.getInstance();
		 ArrayList<com.google.api.services.drive.model.File> files =  server.ListFileMetaDataInDrive(f.getId());
		 for(com.google.api.services.drive.model.File thisFile : files){
			 System.out.println("checking Title"+thisFile.getTitle()+"\t"+LogDataType.FILE_NAME);
			 if(thisFile.getTitle().equals(LogDataType.FILE_NAME)){
				 return true;
			 }
		 }
		return false;
	}

	/**
	 * attempts to upload all logs that have not been updated on ther server yet
	 * @return
	 * @throws IOException
	 */
	private boolean uploadLogs() throws IOException{
		for(LogDataType log : logData.values()){
			if(!log.isUpToDateOnServer()){
				uploadLog(log);
			}
		}
		return false;
	}
	
}
