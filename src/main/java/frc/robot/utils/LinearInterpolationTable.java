package frc.robot.utils;

import java.awt.geom.Point2D;

/**
 * Simple linear interpolation table (piecewise).
 * Given an input x, it returns a linearly interpolated y between the nearest points.
 */
public class LinearInterpolationTable {
    private double maxInput = Double.NEGATIVE_INFINITY;
    private double minInput = Double.POSITIVE_INFINITY;
    private final Point2D[] points;
    public final int size;

    public LinearInterpolationTable(Point2D... points) {
        this.points = points;
        size = this.points.length;
        for (int i = 0; i < size; i++) {
            if (this.points[i].getX() > maxInput) {
                maxInput = this.points[i].getX();
            }
            if (this.points[i].getX() < minInput) {
                minInput = this.points[i].getX();
            }
        }
    }

    public double getOutput(double input) {
        int index = 0;
        if (input <= minInput) {
            index = 0;
        } else if (input >= maxInput) {
            index = size - 2;
        } else {
            for (int i = 1; i < points.length; i++) {
                if (input > points[i - 1].getX() && input <= points[i].getX()) {
                    index = i - 1;
                }
            }
        }
        return interpolate(input, points[index], points[index + 1]);
    }

    public static double interpolate(double input, Point2D point1, Point2D point2) {
        final double slope = (point2.getY() - point1.getY()) / (point2.getX() - point1.getX());
        final double deltaX = input - point1.getX();
        final double deltaY = deltaX * slope;
        return point1.getY() + deltaY;
    }
}
