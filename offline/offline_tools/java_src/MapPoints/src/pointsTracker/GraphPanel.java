package pointsTracker;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Component;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.Rectangle;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseMotionAdapter;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import javax.imageio.ImageIO;
import javax.swing.Icon;
import javax.swing.JComponent;

/**
 * @author John B. Matthews; distribution per GPL.
 * @modified_by Vivaan Bahl, originally taken from https://sites.google.com/site/drjohnbmatthews/graphpanel
 */
public class GraphPanel extends JComponent {

    public int WIDE;
    public int HIGH;
    private static final int RADIUS = 35;
    private static final Random rnd = new Random();
    private int radius = RADIUS;
    private Kind kind = Kind.Circular;
    private List<Node> nodes = new ArrayList<Node>();
    private List<Pin> pins = new ArrayList<Pin>();
    private List<Node> selected = new ArrayList<Node>();
    private List<Edge> edges = new ArrayList<Edge>();
    private Point mousePt = new Point(WIDE / 2, HIGH / 2);
    private Rectangle mouseRect = new Rectangle();
    private boolean selecting = false;
    private Color lineColor = Color.YELLOW;
    private int pinMode;
    
    private static final double LATLNG_DELTA_Y = 0.0000035; //what change in longitude we see from a 1 pixel y delta
    private static final double LATLNG_DELTA_X = 0.0000238; //what change in latitude we see from a 1 pixel x delta
    private static final Point SCHENLEY_STOP_SIGN = new Point(679, 239);
    private static final double SCHENLEY_STOP_SIGN_LATLNG_X = 40.440148;
    private static final double SCHENLEY_STOP_SIGN_LATLNG_Y = -79.942296;
    
    public static final int PIN_MODE_START = 0;
    public static final int PIN_MODE_MID = 1;
    public static final int PIN_MODE_END = 2;
    public static final int PIN_MODE_OOPS = -1;
    


    public GraphPanel(int w, int h) {
    	WIDE = w;
    	HIGH = h;
        this.setOpaque(true);
        this.addMouseListener(new MouseHandler());
        this.addMouseMotionListener(new MouseMotionHandler());
        this.setSize(WIDE, HIGH);
        pinMode = GraphPanel.PIN_MODE_OOPS;
    }
    
    public List<Pin> getPins() {
    	return pins;
    }
    
    @Override
    public void paintComponent(Graphics g) {
    	Graphics2D g2 = (Graphics2D)g;
    	BufferedImage img = null;
        try {
			img = ImageIO.read(ClassLoader.getSystemResourceAsStream("courseMap.png"));
		} catch (IOException e1) {
			// TODO Auto-generated catch block
			System.out.println("OOPS OOPS OOPS BAD STUFF HAPPENING");
		}
    	
        g.setColor(new Color(0x00f0f0f0));
        g.fillRect(0, 0, getWidth(), getHeight());
        g2.drawImage(img, null, 0, 0);
        drawLines(g2);
        for (Edge e : edges) {
            e.draw(g);
        }
        for (Node n : nodes) {
            n.draw(g);
            g.drawString("" + nodes.indexOf(n), n.getLocation().x + 10, n.getLocation().y + 10);
        }
        if (selecting) {
            g.setColor(Color.darkGray);
            g.drawRect(mouseRect.x, mouseRect.y,
                mouseRect.width, mouseRect.height);
        }
        
        
        
    }
    
    public void addPin(Pin p) {
    	int indexToAdd = 0;
    	
    	if (nodes.size() > 0) {
    		indexToAdd = nodes.size() - 1;
    		if (p instanceof StartPin) {
    			indexToAdd = 0;
    			if (pins.get(0) instanceof StartPin) {
    				pins.remove(0);
    				nodes.remove(0);
    			}
    			pins.add(indexToAdd, p);
    	        nodes.add(indexToAdd, new Node(new Point(p.getScreenX(), p.getScreenY()), p.getPinWidth(), p.getPinColor(), Kind.Circular));
    	        // this is the only one that doesn't append to the end
    	        ((Display)this.getParent()).updatePinList();
    	        return;
    		}
        	
        	else if(p instanceof EndPin) {
        		if(pins.get(indexToAdd) instanceof EndPin) {
        			pins.remove(indexToAdd);
        			nodes.remove(indexToAdd);
        		}
        	}
		}
    	
    	pins.add(p);
        nodes.add(new Node(new Point(p.getScreenX(), p.getScreenY()), p.getPinWidth(), p.getPinColor(), Kind.Circular));
        
        for (int i = 0; i < pins.size(); i++) {
			if (pins.get(i) instanceof EndPin) {
				pins.add(pins.remove(i));
				nodes.add(nodes.remove(i));
				break;
			}
		}
        
        ((Display)this.getParent()).updatePinList();
        repaint();
    }

    private void drawLines(Graphics2D g2) {
		// TODO Auto-generated method stub
    	g2.setColor(lineColor);
    	g2.setStroke(new BasicStroke(3));
		for (int i = 1; i < pins.size(); i++) {
			Node n1 = nodes.get(i);
			Node n2 = nodes.get(i - 1);
			g2.drawLine(n1.getLocation().x, n1.getLocation().y, n2.getLocation().x, n2.getLocation().y);
		}
		g2.setStroke(new BasicStroke(2));
	}

    
    
    public void setPinMode(int setTo) {
    	pinMode = setTo;
    }
    
    public void removeSelected() {
    	selected = Node.getSelected(nodes, new ArrayList<Node>());
    	for(int i = 0; i < selected.size(); i++) {
    		pins.remove(nodes.indexOf(selected.get(i)));
			nodes.remove(selected.get(i));
	
			
			repaint();
		}
    	selected = new ArrayList<Node>();
    	repaint();
    }
    
    
    private double[] latLngForPoint(Point toCompare) {
    	int dx = SCHENLEY_STOP_SIGN.x - toCompare.x;
        int dy = SCHENLEY_STOP_SIGN.y - toCompare.y;
        
        double pinLat = SCHENLEY_STOP_SIGN_LATLNG_X - LATLNG_DELTA_X * (dx);
        double pinLon = SCHENLEY_STOP_SIGN_LATLNG_Y - LATLNG_DELTA_Y * dy;
        double[] list = new double[2];
        list[0] = pinLat;
        list[1] = pinLon;
        return list;
    }
    
    //system code below...don't touch this
    
    
    
    
    
	private class MouseHandler extends MouseAdapter {

        @Override
        public void mouseReleased(MouseEvent e) {
            selecting = false;
            mouseRect.setBounds(0, 0, 0, 0);
            e.getComponent().repaint();
            ((Display)GraphPanel.this.getParent()).updatePinList();
        }

        @Override
        public void mousePressed(MouseEvent e) {
            mousePt = e.getPoint();
            
            if (e.isShiftDown()) {
                Node.selectToggle(nodes, mousePt);
            } else if (e.isPopupTrigger()) {
                Node.selectOne(nodes, mousePt);
            } else if (Node.selectOne(nodes, mousePt)) {
                selecting = false;
            } else {
                Node.selectNone(nodes);
                selecting = true;
                
                
                double[] list = latLngForPoint(mousePt);
                double pinLat = list[0];
                double pinLon = list[1];
                
                
                switch (pinMode) {
				case GraphPanel.PIN_MODE_START:
					addPin(new StartPin(mousePt.x, mousePt.y, pinLat, pinLon));
					break;
					
				case GraphPanel.PIN_MODE_MID:
					addPin(new Pin(mousePt.x, mousePt.y, pinLat, pinLon));
					break;
					
				case GraphPanel.PIN_MODE_END:
					addPin(new EndPin(mousePt.x, mousePt.y, pinLat, pinLon));
					break;

				default:
					System.out.println("OOPS OOPS OOPS");
					break;
				}
            }
            e.getComponent().repaint();
        }
    }

    private class MouseMotionHandler extends MouseMotionAdapter {

        Point delta = new Point();

        @Override
        public void mouseDragged(MouseEvent e) {
            if (selecting) {
                mouseRect.setBounds(
                    Math.min(mousePt.x, e.getX()),
                    Math.min(mousePt.y, e.getY()),
                    Math.abs(mousePt.x - e.getX()),
                    Math.abs(mousePt.y - e.getY()));
                Node.selectRect(nodes, mouseRect);
            } else {
                delta.setLocation(
                    e.getX() - mousePt.x,
                    e.getY() - mousePt.y);
                Node.updatePosition(nodes, delta);
                updatePinPositions();
                mousePt = e.getPoint();
            }
            e.getComponent().repaint();
        }

		private void updatePinPositions() {
			// TODO Auto-generated method stub
			for(Node n : nodes) {
				Point point = n.getLocation();
				int index = nodes.indexOf(n);
				Pin p = pins.get(index);
				p.setPinLocation(point);
				double[] list = latLngForPoint(point);
                double pinLat = list[0];
                double pinLon = list[1];
                p.setPinLatLng(pinLat, pinLon);
				repaint();
			}
		}
    }

    
    /**
     * The kinds of node in a graph.
     */
    private enum Kind {

        Circular, Rounded, Square;
    }

    /**
     * An Edge is a pair of Nodes.
     */
    private static class Edge {

        private Node n1;
        private Node n2;

        public Edge(Node n1, Node n2) {
            this.n1 = n1;
            this.n2 = n2;
        }

        public void draw(Graphics g) {
            Point p1 = n1.getLocation();
            Point p2 = n2.getLocation();
            g.setColor(Color.darkGray);
            g.drawLine(p1.x, p1.y, p2.x, p2.y);
        }
    }

    /**
     * A Node represents a node in a graph.
     */
    private static class Node {

        private Point p;
        private int r;
        private Color color;
        private Kind kind;
        private boolean selected = false;
        private Rectangle b = new Rectangle();

        /**
         * Construct a new node.
         */
        public Node(Point p, int r, Color color, Kind kind) {
            this.p = p;
            this.r = r;
            this.color = color;
            this.kind = kind;
            setBoundary(b);
        }

        /**
         * Calculate this node's rectangular boundary.
         */
        private void setBoundary(Rectangle b) {
            b.setBounds(p.x - r, p.y - r, 2 * r, 2 * r);
        }

        /**
         * Draw this node.
         */
        public void draw(Graphics g) {
            g.setColor(this.color);
            if (this.kind == Kind.Circular) {
                g.fillOval(b.x, b.y, b.width, b.height);
            } else if (this.kind == Kind.Rounded) {
                g.fillRoundRect(b.x, b.y, b.width, b.height, r, r);
            } else if (this.kind == Kind.Square) {
                g.fillRect(b.x, b.y, b.width, b.height);
            }
            if (selected) {
                g.setColor(Color.darkGray);
                g.drawRect(b.x, b.y, b.width, b.height);
            }
        }

        /**
         * Return this node's location.
         */
        public Point getLocation() {
            return p;
        }

        /**
         * Return true if this node contains p.
         */
        public boolean contains(Point p) {
            return b.contains(p);
        }

        /**
         * Return true if this node is selected.
         */
        public boolean isSelected() {
            return selected;
        }

        /**
         * Mark this node as selected.
         */
        public void setSelected(boolean selected) {
            this.selected = selected;
        }

        /**
         * Collected all the selected nodes in list.
         */
        public static List<Node> getSelected(List<Node> list, List<Node> selected) {
            selected.clear();
            for (Node n : list) {
                if (n.isSelected()) {
                    selected.add(n);
                }
            }
            return selected;
        }

        /**
         * Select no nodes.
         */
        public static void selectNone(List<Node> list) {
            for (Node n : list) {
                n.setSelected(false);
            }
        }

        /**
         * Select a single node; return true if not already selected.
         */
        public static boolean selectOne(List<Node> list, Point p) {
            for (Node n : list) {
                if (n.contains(p)) {
                    if (!n.isSelected()) {
                        Node.selectNone(list);
                        n.setSelected(true);
                    }
                    return true;
                }
            }
            return false;
        }

        /**
         * Select each node in r.
         */
        public static void selectRect(List<Node> list, Rectangle r) {
            for (Node n : list) {
                n.setSelected(r.contains(n.p));
            }
        }

        /**
         * Toggle selected state of each node containing p.
         */
        public static void selectToggle(List<Node> list, Point p) {
            for (Node n : list) {
                if (n.contains(p)) {
                    n.setSelected(!n.isSelected());
                }
            }
        }

        /**
         * Update each node's position by d (delta).
         */
        public static void updatePosition(List<Node> list, Point d) {
            for (Node n : list) {
                if (n.isSelected()) {
                    n.p.x += d.x;
                    n.p.y += d.y;
                    n.setBoundary(n.b);
                }
            }
        }

        /**
         * Update each node's radius r.
         */
        public static void updateRadius(List<Node> list, int r) {
            for (Node n : list) {
                if (n.isSelected()) {
                    n.r = r;
                    n.setBoundary(n.b);
                }
            }
        }

        /**
         * Update each node's color.
         */
        public static void updateColor(List<Node> list, Color color) {
            for (Node n : list) {
                if (n.isSelected()) {
                    n.color = color;
                }
            }
        }

        /**
         * Update each node's kind.
         */
        public static void updateKind(List<Node> list, Kind kind) {
            for (Node n : list) {
                if (n.isSelected()) {
                    n.kind = kind;
                }
            }
        }
    }

    private static class ColorIcon implements Icon {

        private static final int WIDE = 20;
        private static final int HIGH = 20;
        private Color color;

        public ColorIcon(Color color) {
            this.color = color;
        }

        public Color getColor() {
            return color;
        }

        public void setColor(Color color) {
            this.color = color;
        }

        public void paintIcon(Component c, Graphics g, int x, int y) {
            g.setColor(color);
            g.fillRect(x, y, WIDE, HIGH);
        }

        public int getIconWidth() {
            return WIDE;
        }

        public int getIconHeight() {
            return HIGH;
        }
    }

	
}