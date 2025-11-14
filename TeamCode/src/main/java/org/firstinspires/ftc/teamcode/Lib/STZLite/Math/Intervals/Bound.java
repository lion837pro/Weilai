package org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Intervals;

import org.firstinspires.ftc.teamcode.Lib.STZLite.Geometry.Pose;
import org.firstinspires.ftc.teamcode.Lib.STZLite.Geometry.Translation;

/**
 * The {@code Bound2D} class defines an axis-aligned rectangle in the XY plane,
 * represented by two 1D domains (intervals in X and Y).
 * It also supports tolerance-based construction around a center point.
 */
public class Bound{

    private Interval Xaxis;
    private Interval Yaxis;

    private double tolerance;

    private Translation translation;

    /**
     * Constructs a 2D space based on the X and Y coordinates with a given tolerance.
     * The tolerance defines a percentage range around the coordinates.
     *
     * @param x The X-coordinate.
     * @param y The Y-coordinate.
     * @param tolerance The allowable deviation in both X and Y axes (percentage of the coordinates).
     */
    public Bound(double x, double y, double tolerance){

        this.tolerance = tolerance;
        this.translation = new Translation(x, y);

        double percentageInX = Math.abs(x * tolerance);
        double percentageInY = Math.abs(y * tolerance);

        Xaxis = new Interval(x - percentageInX, x + percentageInX);
        Yaxis = new Interval(y - percentageInY, y + percentageInY);
    }

    /**
     * Constructs a 2D space based on the minimum and maximum bounds for both X and Y axes.
     * The center of the rectangle is calculated as the midpoint of the provided bounds.
     * The tolerance is set to zero in this case.
     *
     * @param minX The minimum X-coordinate.
     * @param maxX The maximum X-coordinate.
     * @param minY The minimum Y-coordinate.
     * @param maxY The maximum Y-coordinate.
     * @return A new {@code Rectangle2D} instance representing the defined space.
     */
    public static Bound fromLimits(double minX, double maxX, double minY, double maxY) {
        return new Bound(
                new Translation((minX + maxX) / 2.0, (minY + maxY) / 2.0),
                0
        );
    }

    /**
     * Constructs a 2D space based on a {@code Translation2d} object and a given tolerance.
     *
     * @param center The 2D coordinate (X and Y) as a {@code Translation2d} object.
     * @param tolerance The allowable deviation in both X and Y axes (percentage of the coordinates).
     */
    public Bound(Translation center, double tolerance){

        this.tolerance = tolerance;
        this.translation = center;

        double dX = center.getX() * tolerance;
        double dY = center.getY() * tolerance;

        Xaxis = new Interval(center.getX() - dX, center.getX() + dX);
        Yaxis = new Interval(center.getY() - dY, center.getY() + dY);

    }

    /**
     * Returns the center of the bound.
     */
    public Translation getCenter(){
        return translation;
    }

    /**
     * Checks if this bound overlaps with another {@code Bound2D}.
     * @param other The other {@code Bound2D} to check against.
     * @return {@code true} if the bounds overlap, {@code false} otherwise.
     */
    public boolean overlaps(Bound other) {
        return Xaxis.overlaps(other.Xaxis) && Yaxis.overlaps(other.Yaxis);
    }


    /**
     * Expands the bound by a fixed amount on each side.
     * @param amount the amount to expand the bound by
     */
    public void expand(double amount){
        Xaxis.expand(amount);
        Yaxis.expand(amount);
    }

    /**
     * Expands the bound by a percentage of its current size on each side.
     * @param percent the percentage to expand the bound by (e.g., 0.1 for 10%)
     */
    public void expandPercent(double percent){
        Xaxis.expandPercent(percent);
        Yaxis.expandPercent(percent);
    }


    /**
     * Sets the center of the bound to a new {@code Translation2d} point and updates the intervals based on the current tolerance.
     * @param center
     */
    public void setCenter(Translation center){
        this.translation = center;

        double dX = center.getX() * tolerance;
        double dY = center.getY() * tolerance;

        Xaxis.setLimits(center.getX() - dX, center.getX() + dX);
        Yaxis.setLimits(center.getY() - dY, center.getY() + dY);

    }

    /** Returns the tolerance used to build this bound. */
    public double getTolerance(){
        return tolerance;
    }

    /**
     * Returns the range of X (min and max) for this bound as a {@code Translation2d}.
     *
     * @return The range of X.
     */
    public Translation getXasPoint(){
        return new Translation(Xaxis.getMin(), Xaxis.getMax());
    }

    /**
     * Returns the range of Y (min and max) for this bound as a {@code Translation2d}.
     *
     * @return The range of Y.
     */
    public Translation getYasPoint(){
        return new Translation(Yaxis.getMin(), Yaxis.getMax());
    }

    /** Returns the X interval of this bound. */
    public Interval getXInterval(){
        return Xaxis;
    }

    /** Returns the Y interval of this bound. */
    public Interval getYInterval(){
        return Yaxis;
    }

    /**
     * Checks if a given {@code Pose2d} is within the defined 2D space.
     *
     * @return {@code true} if the pose is within the space, {@code false} otherwise.
     */
    public boolean contains(Translation point){
        return Xaxis.isInRange(point.getX()) && Yaxis.isInRange(point.getY());
    }

    /** Checks if the given pose lies inside the bound. */
    public boolean contains(Pose pose) {
        return contains(pose.getTranslation());
    }

    /** Checks if the given x and y coordinates lie inside the bound. */
    public boolean contains(double x, double y) {
        return Xaxis.isInRange(x) && Yaxis.isInRange(y);
    }

}