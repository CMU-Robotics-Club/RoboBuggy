package com.roboclub.robobuggy.jetty.gui;

import org.eclipse.jetty.server.Handler;
import org.eclipse.jetty.server.Server;
import org.eclipse.jetty.server.handler.ContextHandler;
import org.eclipse.jetty.server.handler.ContextHandlerCollection;
import org.eclipse.jetty.server.handler.DefaultHandler;
import org.eclipse.jetty.server.handler.HandlerList;
import org.eclipse.jetty.server.handler.ResourceHandler;
import org.eclipse.jetty.servlets.gzip.GzipHandler;
import org.eclipse.jetty.websocket.server.WebSocketHandler;
import org.eclipse.jetty.websocket.servlet.WebSocketServletFactory;

public class JettyServer {
	
	private Server server;
	private Thread serverThread;
	
	public JettyServer() throws Exception {
		
		System.out.println(System.getProperty("user.dir"));
		server = new Server(8080);
		
		
		// Have something to publish to all of the connected clients
		ClientUpdater cu = new ClientUpdater();
		
		// Root handler for HTML
		ResourceHandler res = new ResourceHandler();
		res.setDirectoriesListed(true);
		res.setWelcomeFiles(new String[]{ "index.html", "googleMaps.js", "scripts.js", "speedometer.js"});
		res.setResourceBase("../../Web GUI");
		
		// Root handler for WebSockets
		WebSocketHandler wsHandler = new WebSocketHandler() {
            @Override
            public void configure(WebSocketServletFactory factory) {
                factory.register(WSHandler.class);
            }
        };
                
        // Root handler collection
        GzipHandler gzip = new GzipHandler();
        HandlerList handlers = new HandlerList();
        handlers.setHandlers(new Handler[] {wsHandler, res, new DefaultHandler()});
        gzip.setHandler(handlers);
        
        // Root context manager
        ContextHandler contextRoot = new ContextHandler("/");
        contextRoot.setHandler(gzip);
        
        
        // Help handler for HTML
        RootHandler help = new RootHandler();
        
        // Help context manager
        ContextHandler contextHelp = new ContextHandler("/dicks");
        contextHelp.setHandler(help);
        
        
        // Aggregate the various contexts
        ContextHandlerCollection contexts = new ContextHandlerCollection();
        contexts.setHandlers(new Handler[] {contextRoot, contextHelp, new DefaultHandler()});
        
        // Set the server appropriately for those contexts
        server.setHandler(contexts);        
        
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
