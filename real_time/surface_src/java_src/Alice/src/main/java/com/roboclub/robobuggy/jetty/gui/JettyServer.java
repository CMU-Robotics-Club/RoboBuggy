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

/**
 * a server for hosting the GUI and delivering
 */
public class JettyServer {
	
	private Server server;
	private Thread serverThread;

    /**
     * initializes the jetty server
     */
    public JettyServer() {
		
		System.out.println(System.getProperty("user.dir"));
		server = new Server(8080);
		
		
		// Have something to publish to all of the connected clients
		new ClientUpdater();
		
		// Root handler for HTML
		ResourceHandler res = new ResourceHandler();
		res.setDirectoriesListed(true);
		res.setWelcomeFiles(new String[]{ "index.html"});
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
        
        // Static handler for test
        ResourceHandler res1 = new ResourceHandler();
        res1.setWelcomeFiles(new String[]{"index.html"});
        res1.setResourceBase("/home/mint/Downloads/adf-dynamic-example/public");
        
        ContextHandler contextTest = new ContextHandler("/test");
        contextTest.setHandler(res1);
        
        // Static handler for test
        ResourceHandler res11 = new ResourceHandler();
        res11.setWelcomeFiles(new String[]{"ws.html", "ws.js"});
        res11.setResourceBase("../../Web GUI");
        
        ContextHandler contextTest1 = new ContextHandler("/basic");
        contextTest1.setHandler(res11);
        
        // Aggregate the various contexts
        ContextHandlerCollection contexts = new ContextHandlerCollection();
        contexts.setHandlers(new Handler[] {contextRoot, contextHelp, contextTest, contextTest1, new DefaultHandler()});
        
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
