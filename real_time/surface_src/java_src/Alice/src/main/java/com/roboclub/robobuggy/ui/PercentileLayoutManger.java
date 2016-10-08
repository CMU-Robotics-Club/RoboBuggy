package com.roboclub.robobuggy.ui;

import java.awt.Component;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.LayoutManager;
import java.util.ArrayList;

/**
 * A class that helps manage adding components with percentages
 */
class PercentileLayoutManger implements LayoutManager {
    private ArrayList<ComponentData> components;

    PercentileLayoutManger(ArrayList<ComponentData> components) {
        this.components = components;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void addLayoutComponent(String name, Component comp) {
        // TODO Auto-generated method stub

    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void removeLayoutComponent(Component comp) {
        // TODO Auto-generated method stub

    }

    /**
     * {@inheritDoc}
     */
    @Override
    public Dimension preferredLayoutSize(Container parent) {
        // TODO Auto-generated method stub
        return null;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public Dimension minimumLayoutSize(Container parent) {
        // TODO Auto-generated method stub
        return null;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void layoutContainer(Container parent) {
        int frameWidth = parent.getWidth();
        int frameHeight = parent.getHeight();
        for (int i = 0; i < this.components.size(); i++) {
            Component thisComponent = this.components.get(i).getComponent();
            int x = (int) (this.components.get(i).getPercentageLeft() * frameWidth);
            int y = (int) (this.components.get(i).getPercentageTop() * frameHeight);
            int width = (int) (this.components.get(i).getPercentageWidth() * frameWidth);
            int height = (int) (this.components.get(i).getPercentageHeight() * frameHeight);
            thisComponent.setBounds(x, y, width, height);
        }
    }

}