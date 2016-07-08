package com.roboclub.robobuggy.ui;

import java.awt.Component;

/**
 * Class for storing and passing around the gui component data
 */
public class ComponentData {
    private Component component;
    private double percentageLeft;
    private double percentageTop;
    private double percentageWidth;
    private double percentageHeight;

    /**
     * Construct a new {@link ComponentData} object
     *
     * @param component        {@link Component} it represents
     * @param percentageLeft   percentage from the left side to start the component
     * @param percentageTop    percentage from the top to start the component
     * @param percentageWidth  width of the component in percent of the parent
     * @param percentageHeight height of the component in percent of the parent
     */
    public ComponentData(Component component,
                         double percentageLeft,
                         double percentageTop,
                         double percentageWidth,
                         double percentageHeight) {
        this.component = component;
        this.percentageLeft = percentageLeft;
        this.percentageTop = percentageTop;
        this.percentageWidth = percentageWidth;
        this.percentageHeight = percentageHeight;
    }

    /**
     * Returns the {@link Component} of the {@link ComponentData}
     *
     * @return the {@link Component} of the {@link ComponentData}
     */
    public Component getComponent() {
        return component;
    }

    /**
     * Returns the percentage from the left of the {@link ComponentData}
     *
     * @return the percentage from the left of the {@link ComponentData}
     */
    public double getPercentageLeft() {
        return percentageLeft;
    }

    /**
     * Returns the percentage from the top of the {@link ComponentData}
     *
     * @return the percentage from the top of the {@link ComponentData}
     */
    public double getPercentageTop() {
        return percentageTop;
    }

    /**
     * Returns the percentage width of the {@link ComponentData}
     *
     * @return the percentage width of the {@link ComponentData}
     */
    public double getPercentageWidth() {
        return percentageWidth;
    }

    /**
     * Returns the percentage height of the {@link ComponentData}
     *
     * @return the percentage height of the {@link ComponentData}
     */
    public double getPercentageHeight() {
        return percentageHeight;
    }
}