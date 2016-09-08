package com.roboclub.robobuggy.nodes.sensors;

import com.roboclub.robobuggy.nodes.baseNodes.BuggyDecoratorNode;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyNode;

/**
 * A node to communicate with the Hillcrest Freespace 9DOF IMU
 */
public class HillcrestNode extends BuggyDecoratorNode {

    /**
     * Creates a new decorator for the given {@link Node}
     *
     * @param node {@link Node} to decorate
     * @param name the name we want for this node to store so that it can be referenced later
     */
    public HillcrestNode(BuggyNode node, String name) {
        super(node, name);
    }

    @Override
    protected boolean startDecoratorNode() {
        return false;
    }

    @Override
    protected boolean shutdownDecoratorNode() {
        return false;
    }
}
