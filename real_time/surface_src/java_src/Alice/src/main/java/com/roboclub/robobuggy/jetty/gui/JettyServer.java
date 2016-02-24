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
		
		server = new Server(8080);
		
		// Have something to publish data to all of the connected clients
		ClientDataUpdater cu = new ClientDataUpdater();
		ClientImageUpdater ciu = new ClientImageUpdater();
		
		// Root handler for HTML
		ResourceHandler res = new ResourceHandler();
		res.setDirectoriesListed(true);
		res.setWelcomeFiles(new String[]{ "index.html"});
		res.setResourceBase("../../Web GUI/root");
		
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
        ContextHandler contextRoot = new ContextHandler("/");
        contextRoot.setHandler(gzip);
                
        // Static handler for test
        ResourceHandler basicRes= new ResourceHandler();
        basicRes.setWelcomeFiles(new String[]{"ws.html"});
        basicRes.setResourceBase("../../Web GUI/basic");
        ContextHandler basicContext = new ContextHandler("/basic");
        basicContext.setHandler(basicRes);
        
        // Static handler for test
        ResourceHandler newRes = new ResourceHandler();
        newRes.setWelcomeFiles(new String[]{"index.html"});
        newRes.setResourceBase("../../Web GUI/new");
        ContextHandler newContext = new ContextHandler("/new");
        newContext.setHandler(newRes);
        
        // Aggregate the various contexts
        ContextHandlerCollection contexts = new ContextHandlerCollection();
        contexts.setHandlers(new Handler[] {contextRoot, basicContext, newContext, new DefaultHandler()});
        
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
