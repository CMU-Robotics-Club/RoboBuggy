package com.roboclub.robobuggy.logging.autoLogging;

/*
 * Modified by Trevor Decker
 * Copyright (c) 2012 Google Inc.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except
 * in compliance with the License. You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software distributed under the License
 * is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express
 * or implied. See the License for the specific language governing permissions and limitations under
 * the License.
 */

//TODO add check for Internet connection
//TODO clean up this class

import com.google.api.client.auth.oauth2.Credential;
import com.google.api.client.extensions.java6.auth.oauth2.AuthorizationCodeInstalledApp;
import com.google.api.client.extensions.jetty.auth.oauth2.LocalServerReceiver;
import com.google.api.client.googleapis.auth.oauth2.GoogleAuthorizationCodeFlow;
import com.google.api.client.googleapis.auth.oauth2.GoogleClientSecrets;
import com.google.api.client.googleapis.javanet.GoogleNetHttpTransport;
import com.google.api.client.googleapis.media.MediaHttpDownloader;
import com.google.api.client.googleapis.media.MediaHttpUploader;
import com.google.api.client.http.FileContent;
import com.google.api.client.http.GenericUrl;
import com.google.api.client.http.HttpTransport;
import com.google.api.client.json.JsonFactory;
import com.google.api.client.json.jackson2.JacksonFactory;
import com.google.api.client.util.store.FileDataStoreFactory;
import com.google.api.services.drive.Drive;
import com.google.api.services.drive.DriveScopes;
import com.google.api.services.drive.model.File;
import com.google.api.services.drive.model.ParentReference;
import com.google.api.services.drive.Drive.Children;
import com.google.api.services.drive.model.ChildList;
import com.google.api.services.drive.model.ChildReference;
import com.roboclub.robobuggy.logging.LogDataType;
import com.roboclub.robobuggy.main.MESSAGE_LEVEL;
import com.roboclub.robobuggy.main.RobobuggyLogicException;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class ServerCommunication {
	// Be sure to specify the name of your application. If the application name
	// is {@code null} or
	// blank, the application will log a warning. Suggested format is
	// "MyCompany-ProductName/1.0".

	private static final String APPLICATION_NAME = "Robotic_Buggy";

	// / Directory to store user credentials.
	private static final java.io.File DATA_STORE_DIR = new java.io.File(
			System.getProperty("user.home"), ".store/drive_sample");

	// / Global instance of the {@link DataStoreFactory}. The best practice is
	// to make it a single
	// / globally shared instance across your application.
	private static FileDataStoreFactory dataStoreFactory;

	// / Global instance of the HTTP transport.
	private static HttpTransport httpTransport;

	// / Global instance of the JSON factory.
	private static final JsonFactory JSON_FACTORY = JacksonFactory
			.getDefaultInstance();

	// / Global Drive API client.
	private static Drive drive;

	/**
	 * Constructor for Server communication, will only be available inside this class to fit singleton pattern
	 */
	private ServerCommunication() {
		// constructor for this class
		System.out.println("Working directory" + System.getProperty("user.dir"));

		try {
			httpTransport = GoogleNetHttpTransport.newTrustedTransport();
			dataStoreFactory = new FileDataStoreFactory(DATA_STORE_DIR);
			// authorization
			Credential credential = authorize();
			// set up the global Drive instance
			drive = new Drive.Builder(httpTransport, JSON_FACTORY, credential)
					.setApplicationName(APPLICATION_NAME).build();

			return;
		} catch (IOException e) {
			System.err.println(e.getMessage());
		} catch (Throwable t) {
			t.printStackTrace();
		}
		System.exit(1);
	}

	/**
	 * Authorizes the installed application to access user's protected data.
	 * @return
	 * @throws Exception
	 */
	private static Credential authorize() throws Exception {
		// load client secrets
		GoogleClientSecrets clientSecrets = GoogleClientSecrets.load(
				JSON_FACTORY,
				new InputStreamReader(ServerCommunication.class
						.getResourceAsStream("client_secrets.json")));
		if (clientSecrets.getDetails().getClientId().startsWith("Enter")
				|| clientSecrets.getDetails().getClientSecret()
						.startsWith("Enter ")) {
			System.out
					.println("Enter Client ID and Secret from https://code.google.com/apis/console/?api=drive "
							+ "into drive-cmdline-sample/src/main/resources/client_secrets.json");
			System.exit(1);
		}
		// set up authorization code flow
		GoogleAuthorizationCodeFlow flow = new GoogleAuthorizationCodeFlow.Builder(
				httpTransport, JSON_FACTORY, clientSecrets,
				Collections.singleton(DriveScopes.DRIVE_FILE))
				.setDataStoreFactory(dataStoreFactory).build();
		// authorize
		return new AuthorizationCodeInstalledApp(flow,
				new LocalServerReceiver()).authorize("user");
	}

	/**
	 * 
	 */
	private static ServerCommunication instance = null;

	/**
	 * 
	 * @return
	 */
	public static ServerCommunication getInstance() {
		if (instance == null) {
			instance = new ServerCommunication();
		}
		return instance;
	}
	
	/**
	 * 
	 * @param folderOnServer
	 * @return
	 * @throws IOException
	 */
	public ArrayList<File> ListFileMetaDataInDrive(String folderOnServer) throws IOException{
		return ListTypeMetaDataInDrive(folderOnServer,"application/vnd.google-apps.folder",false);

	}
	
	/**
	 * 
	 * @param folderOnServer
	 * @return
	 * @throws IOException
	 */
	public ArrayList<File> ListFolderMetaDataInDrive(String folderOnServer) throws IOException{
		return ListTypeMetaDataInDrive(folderOnServer,"application/vnd.google-apps.folder",true);

	}
	
	/**
	 *	if accept is true then we will only keep files with data that has a type matching the typeName string
	 *	if accept is false then we will only keep files with data that has a type that does not match the typeName string
	 **/
	private ArrayList<File> ListTypeMetaDataInDrive(String folderOnServer,String typeName,boolean accept) throws IOException{
		ArrayList<File> allMetaData = ListMetaDataInDrive(folderOnServer);
		ArrayList<File> result = new ArrayList<File>();
		for(File thisFile : allMetaData){
			if(thisFile.getMimeType().equals(typeName)){
				if(accept){
					result.add(thisFile);
				}
			}else{
				if(!accept){
					result.add(thisFile);
				}
				
			}
		}
		return result;
	}
	
	/**
	 * 
	 * @param folderOnServer
	 * @return
	 * @throws IOException
	 */
	public ArrayList<File> ListMetaDataInDrive(String folderOnServer) throws IOException{
		ArrayList<File> metaData = new ArrayList<File>();
		ChildList children = getFilesInFolder(folderOnServer);
        for (ChildReference child : children.getItems()) {
        	try{
        		File thisFile = drive.files().get(child.getId()).execute();
  	          metaData.add(thisFile);
        	}catch(IOException e){
        		new RobobuggyLogicException("Trouble downloading a file: "+e.toString(), MESSAGE_LEVEL.EXCEPTION);
        	}
	        }
        return metaData;
	}
	
	  /**
	   * returns a childList of all files belonging to the given folder.
	   *
	   * @param service Drive API service instance.
	   * @param folderId ID of the folder to print files from.
	   */
	  public static ChildList getFilesInFolder(String folderId)
	      throws IOException {
	    Children.List request = drive.children().list(folderId);
	    ChildList children = new ChildList();

	    do {
	      try {
	        ChildList thisPageChildern = request.execute();
	        children.putAll(thisPageChildern);
	        request.setPageToken(thisPageChildern.getNextPageToken());
	      } catch (IOException e) {
	    	  new RobobuggyLogicException("an error occureced geting file in a folder "+e, MESSAGE_LEVEL.EXCEPTION);
	    	  request.setPageToken(null);
	      }
	    } while (request.getPageToken() != null &&
	             request.getPageToken().length() > 0);
	    return children;
	  }

	public static boolean addFolderToDrive(java.io.File FolderToupload,String Drive_folder_Location)
			throws IOException {
		if (drive == null) {
			return false;
		}
		ParentReference root_dir = new ParentReference();
		root_dir.setId(Drive_folder_Location);
		boolean upload_status = uploadFolder(FolderToupload, root_dir);
		if (!upload_status) {
			return false;
		}
		// upload worked

		return true;
	}

	public static boolean uploadFolder(java.io.File inputFile,
			String whereToSave) throws IOException{
		if(inputFile == null){
			new RobobuggyLogicException("tried to upload a null folder refrence", MESSAGE_LEVEL.EXCEPTION);
		}
		
		ParentReference parent = new ParentReference();
		parent.setId(whereToSave);
		return uploadFolder(inputFile,parent);
	}
	
	/**
	 * 
	 * @param inputFile
	 * @param whereToSave
	 * @return return true if upload worked properly, return false if upload did
	 *         not work properly
	 * @throws IOException
	 */
	private static boolean uploadFolder(java.io.File inputFile,
			ParentReference whereToSave) throws IOException {
		if(!inputFile.exists()){
			new RobobuggyLogicException("Tired to upload a folder that does not exsit", MESSAGE_LEVEL.EXCEPTION);
			return false;
		}
		
		String[] files = inputFile.list();
		for (int i = 0; i < files.length; i++) {
			java.io.File thisFile = new java.io.File(
					inputFile.getAbsolutePath() + "/" + files[i]);
			if (!thisFile.isHidden()) {
				if (thisFile.isDirectory()) {
					// upload directory
					File thisFolder = createFolder(false, files[i],
							Arrays.asList(whereToSave));
					ParentReference parent = new ParentReference();
					parent.setId(thisFolder.getId());
					uploadFolder(thisFile, parent);
				} else {
					// upload file
					//will not upload the file if it is the stats file that is added later, to avoid a race condition 
					if(!thisFile.getName().equals(LogDataType.FILE_NAME)){
						uploadFile(false, Arrays.asList(whereToSave), thisFile);
					}
				}
			}
		}
		// TODO provide actual feedback about the folder being uploaded or not.
		return true; 
		}
	
	/**
	 * Returns true if the given string is a valid, not trashed file on drive, false otherwise
	 * @param id
	 * @return
	 */
	public boolean validFileId(String id) {
		//is not a hash so it can not be looked up 
		if(id.equals("")){
			return false;
		}
		
		//attempt to find the hash of the files 
		try {
	        File f = drive.files().get(id).execute();
	        return !f.getLabels().getTrashed();
	    } catch (IOException e) {
	        new RobobuggyLogicException("bad google drive id: " + id, MESSAGE_LEVEL.EXCEPTION);
	    }
	    return false;
	}

	
	public static String createFolder(String title,String whereToSave) throws IOException{
		ParentReference parent = new ParentReference();
		parent.setId(whereToSave);
		File folder = createFolder(true, title, Arrays.asList(parent));
			return folder.getId();
	}

	// create a new folder at a specific location, set parent to be null if you
	// want it to be root
	private static File createFolder(boolean useDirectUpload, String title,
			List<ParentReference> parentList) throws IOException {
		File body = new File();
		body.setTitle(title);
		body.setMimeType("application/vnd.google-apps.folder");
		body.setParents(parentList);
		File file = drive.files().insert(body).execute();
		return file;
	}

	public static File uploadFile(java.io.File inputFile,
			String whereToSave) throws IOException{
		if(inputFile == null){
			new RobobuggyLogicException("tried to upload a null file refrence", MESSAGE_LEVEL.EXCEPTION);
		}
		
		ParentReference parent = new ParentReference();
		parent.setId(whereToSave);
		List<ParentReference> parentList = new ArrayList<ParentReference>();
		parentList.add(parent);
		return uploadFile(true,parentList,inputFile);
	}
	
	/** Uploads a file using either resumable or direct media upload. */
	private static File uploadFile(boolean useDirectUpload,
			List<ParentReference> parentList, java.io.File fileToUpload)
			throws IOException {
		File fileMetadata = new File();
		fileMetadata.setTitle(fileToUpload.getName());
		fileMetadata.setParents(parentList);
		FileContent mediaContent = new FileContent("image/jpeg", fileToUpload);
		Drive.Files.Insert insert = drive.files().insert(fileMetadata,
				mediaContent);
		MediaHttpUploader uploader = insert.getMediaHttpUploader();
		uploader.setDirectUploadEnabled(useDirectUpload);
		uploader.setProgressListener(new FileUploadProgressListener());
		return insert.execute();
	}

	/**
	 * public facing function for downloading a given file from drive 
	 * @param uploadedFile
	 * @param parentDir
	 * @throws IOException
	 */
	//TODO check this
	public static void downloadFile(File uploadedFile,java.io.File parentDir) throws IOException {
		downloadFile(true,uploadedFile,parentDir);
	}
	
	/** Downloads a file using either resumable or direct media download. */
	private static void downloadFile(boolean useDirectDownload,
			File uploadedFile,java.io.File parentDir) throws IOException {
		if (!parentDir.exists() && !parentDir.mkdirs()) {
			throw new IOException("Unable to create parent directory");
		}
		OutputStream out = new FileOutputStream(new java.io.File(parentDir,
				uploadedFile.getTitle()));

		MediaHttpDownloader downloader = new MediaHttpDownloader(httpTransport,
				drive.getRequestFactory().getInitializer());
		downloader.setDirectDownloadEnabled(useDirectDownload);
		downloader.setProgressListener(new FileDownloadProgressListener());
		downloader.download(new GenericUrl(uploadedFile.getDownloadUrl()), out);
	}

	/**
	 * Trashes all of the files and folders inside the given folder on drive
	 * @param logRefrenceOnServer
	 * @throws IOException
	 */
	public void removeContentOfFolder(String logRefrenceOnServer) throws IOException {
		ArrayList<File> files =  ListMetaDataInDrive(logRefrenceOnServer);
		for( File file : files){
			deleteFile(drive,file.getId());
		}
		
	}
	
	  public static void deleteFile(String fileId) {
		  deleteFile(drive,  fileId);
	  }

	
	  /**
	   * Permanently delete a file, skipping the trash.
	   *
	   * @param service Drive API service instance.
	   * @param fileId ID of the file to delete.
	   */
	  private static void deleteFile(Drive service, String fileId) {
	    try {
	      service.files().delete(fileId).execute();
	    } catch (IOException e) {
	      System.out.println("An error occurred: " + e);
	    }
	  }
	  
	  /**
	   * Move a file to the trash.
	   * @param service Drive API service instance.
	   * @param fileId ID of the file to trash.
	   * @return The updated file if successful, {@code null} otherwise.
	   */
	  private static File trashFile(Drive service, String fileId) {
	    try {
	      return service.files().trash(fileId).execute();
	    } catch (IOException e) {
	      System.out.println("An error occurred: " + e);
	    }
	    return null;
	  }

	 /**
	  * makes a copy of what is in the folder on the server into the folder on this computer
	  * @param folderOnThisComptuer
	  * @param folderOnServer
	  * @throws IOException
	  */
	public void downloadFolder(java.io.File folderOnThisComptuer ,
			String folderOnServer) throws IOException {
		
		ArrayList<File> files =  ListFileMetaDataInDrive(folderOnServer);
		ArrayList<File> folders = ListFolderMetaDataInDrive(folderOnServer);

		for(File f : files){
			downloadFile(true,f,folderOnThisComptuer);		
		}
		
		for(File f: folders){
			java.io.File newFolder = new java.io.File(folderOnThisComptuer.getPath()+"/"+f.getTitle());
			newFolder.mkdirs();
			downloadFolder(newFolder,f.getId());
		}
		
		
	}
	  
}
