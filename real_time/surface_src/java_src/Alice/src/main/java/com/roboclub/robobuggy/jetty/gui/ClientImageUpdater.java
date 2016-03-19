package com.roboclub.robobuggy.jetty.gui;

import java.util.Base64;
import java.util.Base64.Encoder;
import java.awt.image.BufferedImage;
import java.io.ByteArrayOutputStream;
import java.nio.ByteBuffer;
import java.util.concurrent.ConcurrentHashMap;
import javax.imageio.ImageIO;

import org.eclipse.jetty.websocket.api.Session;
import com.roboclub.robobuggy.messages.ImageMessage;
import com.roboclub.robobuggy.ros.Message;

/**
 * Updates connected clients with ImageMessages.
 * @author Vasu Agrawal
 *
 */
public class ClientImageUpdater extends ClientUpdater {
	
	private Thread updater;
	
	/**
	 * Subscribe to "camera" group and initialize the things.
	 */
	public ClientImageUpdater() {
		super(); // subscribe to all the things
		WSHandler.getSGM().newGroup("camera");

		updater = new Thread() {
			
			private ConcurrentHashMap<Integer, Session> clients;
			// probably should initialize this with a specific size based on the expected inputs
			// 1.2 is a scaling factor since apparently there's still a resize if the size is exact.
			private ByteArrayOutputStream baos = new ByteArrayOutputStream((int) (1920 * 1080 * 1.2));
			private Message update;
			private Session session;
			private byte[] payload;
			
			private byte[] imageBytes;
			private String imageBase64Encode;
			private Encoder encoder = Base64.getEncoder();
			
			public void run() {
				while (true) {
					try {
						// Since the clients list may have changed from the last time we grabbed it, we'll
						// constantly
						clients = WSHandler.getSGM().getGroup("camera");
						update = updates.take();
						
//						if (clients.isEmpty()) {
//							continue;
//						}
						
						// At the moment, we're passing any and all image messages. In the future, I need
						// to be able to sort by the channel -> make a separate update queue for each channel
						// TODO: Handle multiple camera feeds
						if (update instanceof ImageMessage) {
							// Grab the image
							BufferedImage image = ((ImageMessage) update).getImage();
							
							baos.reset();
							ImageIO.write(image,  "jpg",  baos); // need more jpeg
							baos.flush();

							imageBytes = baos.toByteArray();
							imageBase64Encode = encoder.encodeToString(imageBytes);
							
							/* Header Format
							 * [10 chars] Frame Number
							 * [10 chars] Image Base 64 String Size in bytes
							 */
							String headerString = String.format("%010d%010d",
									((ImageMessage) update).getFrameNumber(),
									imageBase64Encode.length());
							byte[] header = headerString.getBytes("UTF-8");
							byte[] data = imageBase64Encode.getBytes("UTF-8");
														
							payload = new byte[header.length + data.length];
							
							// unnecessary copy, probably better to just use a large autoresizing buffer
							// aka ByteArrayOutputStream
							System.arraycopy(header, 0, payload, 0,             header.length);
							System.arraycopy(data,   0, payload, header.length, data.length);
						} else {
							continue;
						}

						// Send data to clients
						if (clients != null) {
							for (int key: clients.keySet()) {
								session = clients.get(key);
								session.getRemote().sendBytes(ByteBuffer.wrap(payload));
							}
						} else {
							System.out.println("Null clients");
						}
					
					} catch (Exception e) {
						e.printStackTrace();
					}
				}
			}
		};
		
		updater.start();
	}
}