package com.roboclub.robobuggy.jetty.gui;

import org.eclipse.jetty.server.Handler;
import org.eclipse.jetty.server.Server;
import org.eclipse.jetty.server.handler.DefaultHandler;
import org.eclipse.jetty.server.handler.HandlerList;
import org.eclipse.jetty.server.handler.ResourceHandler;
import org.eclipse.jetty.servlets.gzip.GzipHandler;

public class JettyServer {
	
	private Server server;
	private Thread serverThread;
	
	public JettyServer() throws Exception {
		
		System.out.println(System.getProperty("user.dir"));
		server = new Server(8080);
//		server.setHandler(new RootHandler());
		ResourceHandler res = new ResourceHandler();
		res.setDirectoriesListed(true);
		res.setWelcomeFiles(new String[]{ "index.html"});
		res.setResourceBase("../../Web GUI");
		
		GzipHandler gzip = new GzipHandler();
		server.setHandler(gzip);
		HandlerList handlers = new HandlerList();
		handlers.setHandlers(new Handler[] {res, new DefaultHandler()});
		gzip.setHandler(handlers);
		
		serverThread = new Thread(){
			public void run() {
				try {
					server.start();
					server.join();
				} catch (Exception e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		};
		
		serverThread.start();
	}
}
