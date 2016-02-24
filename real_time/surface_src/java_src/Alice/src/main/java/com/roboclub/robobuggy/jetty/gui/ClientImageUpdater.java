package com.roboclub.robobuggy.jetty.gui;

import java.awt.image.BufferedImage;
import java.io.ByteArrayOutputStream;
import java.nio.ByteBuffer;
import java.util.concurrent.ConcurrentHashMap;
import javax.imageio.ImageIO;

import org.eclipse.jetty.websocket.api.Session;
import com.roboclub.robobuggy.messages.ImageMessage;
import com.roboclub.robobuggy.ros.Message;
import com.sun.org.apache.xerces.internal.impl.dv.util.Base64;

public class ClientImageUpdater extends ClientUpdater {
	
	private Thread updater;
	
	public ClientImageUpdater() {
		super(); // subscribe to all the things
		WSHandler.sgm.newGroup("camera");

		updater = new Thread() {
			
			ConcurrentHashMap<Integer, Session> clients;	
			private ByteArrayOutputStream baos = new ByteArrayOutputStream();
			private Message update;
			private Session session;
			private byte[] payload;
			
			public void run() {
				while (true) {
					try {
						clients = WSHandler.sgm.getGroup("camera");
						update = updates.take();
						
						if (update instanceof ImageMessage) {
							// Grab the image
							BufferedImage image = ((ImageMessage) update).getImage();
							
							//convert to jpeg
							baos.reset();
							ImageIO.write(image,  "jpg",  baos);
							baos.flush();

							byte[] bytes = baos.toByteArray();
							String encoded = Base64.encode(bytes);
							byte[] header = String.format("%09d", encoded.length()).getBytes();
							byte[] data = encoded.getBytes();
							
							payload = new byte[header.length + data.length];

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