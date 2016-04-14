package com.roboclub.robobuggy.external;

import java.io.IOException;
import java.io.OutputStreamWriter;
import java.lang.reflect.Modifier;
import java.net.Socket;
import java.net.UnknownHostException;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Subscriber;

/**
 * Pusher to send stuff to the external server
 * @author Vasu Agrawal
 *
 */
public class ExternalPusher {
	
	private Socket socket = null;
	private OutputStreamWriter osw;
	private Gson messageTranslator;
	private String hostname;
	private int port;
	private boolean threadConnecting = false;
	
	/**
	 * Class to push all data to specified hostname and port
	 * @param hostname Hostname of server
	 * @param port Port of server
	 * @throws UnknownHostException thrown if hostname isn't recognized
	 * @throws IOException unable to send over socket
	 */
	public ExternalPusher(String hostname, int port) {
		messageTranslator = new GsonBuilder()
                .excludeFieldsWithModifiers(Modifier.TRANSIENT)
                .serializeSpecialFloatingPointValues()
                .create()
                ;
		this.hostname = hostname;
		this.port = port;
		synchronized(this) {
			reconnect();
		} 
		subscribeToAll();
	}
	 
	/**
	 * Method to subscribe to all channels.
	 */
	private void subscribeToAll() {
		System.out.println("subscribe to all the things");
		MessageListener callback = new MessageListener() {
			
			@Override
			public void actionPerformed(String topicName, Message m) {
            	String str = messageTranslator.toJson(m);
            	try {
					osw.write(str + "\n", 0, str.length() + 1);
					osw.flush();
				} catch (Exception e) {
					/* we make sure that only a single callback can start the
					 * reconnection procedure. This makes sure that there's ample time
					 * to mark threadConnecting, ensuring no race conditions.
					 */
					synchronized(this) {
						reconnect();
					}
				}
			}
		};
				
		// Once we're subscribed to all the things, we shouldn't need to do more other than push.
        for (NodeChannel filter : NodeChannel.getLoggingChannels()) {
            new Subscriber("externalPusher", filter.getMsgPath(), callback);            
            new Subscriber("externalPusher", filter.getStatePath(), callback);
        }
	}
	
	private void reconnect() {
		if (!threadConnecting) {
			threadConnecting = true;
			new Thread(new Runnable() {
				@Override
				public void run() {
					while(true) {
						try {
							Thread.sleep(1000);
							socket = new Socket(hostname, port);
							osw = new OutputStreamWriter(socket.getOutputStream(), "UTF-8");
							threadConnecting = false;
							return;
						} catch (Exception e) {
							new RobobuggyLogicNotification("Attempting to reestablish connection to external server",
									RobobuggyMessageLevel.WARNING);
						}
					}
				}
			}).start();
		}
	}
}