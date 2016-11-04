package com.roboclub.robobuggy.nodes.localizers;

import com.roboclub.robobuggy.nodes.baseNodes.BuggyNode;
import com.roboclub.robobuggy.nodes.baseNodes.PeriodicNode;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;

/**
 * Created by vivaanbahl on 11/4/16.
 */
public class RobobuggyKFLocalizer extends PeriodicNode {
    private Publisher posePub;

    /**
     * Create a new {@link PeriodicNode} decorator
     *
     * @param base   {@link BuggyNode} to decorate
     * @param period of the periodically executed portion of the node
     * @param name
     */
    protected RobobuggyKFLocalizer(BuggyNode base, int period, String name) {
        super(base, period, name);
        posePub = new Publisher(NodeChannel.POSE.getMsgPath());
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
