package com.roboclub.robobuggy.jetty.gui;

import java.util.ArrayList;

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

/**
 * Webserver class running inside the buggy. Only instantiate one.
 * @author Vasu Agrawal
 *
 */
public class JettyServer {
	
	private Server server;
	private Thread serverThread;
	private ArrayList<ClientUpdater> updaters;
	
	/**
	 * Starts a new Jetty instance and manages directories correctly.
	 * @throws Exception unable to start server
	 */
	public JettyServer() throws Exception {
		
		server = new Server(8080);
		
		updaters = new ArrayList<ClientUpdater>();
		updaters.add(new ClientDataUpdater());
		updaters.add(new ClientImageUpdater());

		// Root handler for HTML
		ResourceHandler res = new ResourceHandler();
		res.setDirectoriesListed(true);
		res.setWelcomeFiles(new String[]{ "index.html"});
		res.setResourceBase("../../Web GUI/updated");
		
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
        ResourceHandler cameraRes = new ResourceHandler();
        cameraRes.setWelcomeFiles(new String[]{"index.html"});
        cameraRes.setResourceBase("../../Web GUI/camera");
        ContextHandler cameraContext = new ContextHandler("/camera");
        cameraContext.setHandler(cameraRes);
        
        // Aggregate the various contexts
        ContextHandlerCollection contexts = new ContextHandlerCollection();
        contexts.setHandlers(new Handler[] {contextRoot, cameraContext, new DefaultHandler()});
        
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
