package com.roboclub.robobuggy.simulation;

import com.roboclub.robobuggy.nodes.baseNodes.BuggyBaseNode;
import com.roboclub.robobuggy.nodes.baseNodes.PeriodicNode;
import com.roboclub.robobuggy.ros.NodeChannel;

/**
 * Created by vivaanbahl on 4/2/17.
 */
public class FullSimRunner extends PeriodicNode {
    /**
     * Create a new {@link PeriodicNode} decorator
     *
     * @param period of the periodically executed portion of the node
     * @param name
     */
    protected FullSimRunner(int period, String name) {
        super(new BuggyBaseNode(NodeChannel.SIMULATION), period, name);
    }

    @Override
    protected void update() {

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
