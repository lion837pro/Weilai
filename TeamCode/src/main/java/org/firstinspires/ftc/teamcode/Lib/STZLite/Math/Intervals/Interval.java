package org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Intervals;

/**
 * Represents a range of values
 */
public class Interval{

    /**
     * Represents the domain value
     */
    public enum IntervalLimit{
        MIN, MAX
    }

    public enum IntervalEdge{
        FULL, EMPTY
    }

    private double min, max;

    private final IntervalEdge minEdge;
    private final IntervalEdge maxEdge;

    /**
     * Represents a zone
     * @param min the minimum value
     * @param max the maximum value
     * @throws IllegalArgumentException if the minimum value is greater than the maximum value
     */
    public Interval(double min , double max){

        if (min > max){
            throw new IllegalArgumentException("The minimum value must be less than the maximum value");
        }

        this.min = min;
        this.max = max;

        this.maxEdge = IntervalEdge.FULL;
        this.minEdge = IntervalEdge.FULL;
    }

    public Interval(double min, IntervalEdge minEdge, double max, IntervalEdge maxEdge){

        if (min > max){
            throw new IllegalArgumentException("The minimum value must be less than the maximum value");
        }

        this.min = min;
        this.max = max;

        this.minEdge = minEdge;
        this.maxEdge = maxEdge;
    }

    /**
     * STATIC METHOD
     * Check if a value is in a range
     * @param value the value to check
     * @param min the minimum value
     * @param max the maximum value
     * @return true if the value is in the range
     */
    public static boolean isInRange(double value, double min, double max){

        return value >= min && value <= max;
    }

    /**
     * STATIC METHOD
     * Gets the distance to a point
     * @param value the value to check
     * @param point the point to check
     * @return the distance to the point
     */
    public static double distanceTo(double value, double point) {
        return Math.abs(point - value);
    }

    /**
     * STATIC METHOD
     * Check if a value is out of a range
     * @param value the value to check
     * @param min the minimum value
     * @param max the maximum value
     * @return true if the value is out of the range
     */
    public static boolean outOfRange(double value, double min, double max){
        return !isInRange(value, min, max);
    }

    /**
     * Set the minimum value
     * @param min the minimum value
     * @throws IllegalArgumentException if the minimum value is greater than the maximum value
     */
    public void setMin(double min){
        if (min > this.max) {
            throw new IllegalArgumentException("The minimum value must be less than or equal to the maximum value");
        }
        this.min = min;
    }

    /**
     * Set the maximum value
     * @param max the maximum value
     * @throws IllegalArgumentException if the maximum value is less than the minimum value
     */
    public void setMax(double max){
        if (max < this.min) {
            throw new IllegalArgumentException("The maximum value must be greater than or equal to the minimum value");
        }
        this.max = max;
    }

    /**
     * Set the limits of the interval
     * @param min the minimum value
     * @param max the maximum value
     * @throws IllegalArgumentException if the minimum value is greater than the maximum value
     */
    public void setLimits(double min, double max){

        if (min > max){
            throw new IllegalArgumentException("The minimum value must be less than the maximum value");

        }

        this.min = min;
        this.max = max;
    }

    /**
     * Gets the nearest limit to a value
     * @param value the value to check
     * @return the nearest limit (MIN or MAX)
     */
    public IntervalLimit nearestLimit(double value){
        if(Math.abs(value - min) < Math.abs(value - max)){
            return IntervalLimit.MIN;
        }else{
            return IntervalLimit.MAX;
        }
    }

    /** Returns true if the value is closer to min than max. */
    public boolean isCloserToMin(double value) {
        return Math.abs(value - min) < Math.abs(value - max);
    }

    /** Returns true if the value is closer to max than min. */
    public boolean isCloserToMax(double value) {
        return Math.abs(value - max) < Math.abs(value - min);
    }

    /**
     * Check if a value is at half of the interval
     * @param value the value to check
     * @return true if the value is at half
     */
    public boolean isAtMidpoint(double value){
        return value == min + (max - min) / 2;
    }

    /**
     * Gets the distance to the minimum
     * @param value the value to check
     * @return the distance to the minimum
     */
    public double distanceToMin(double value) {
        return distanceTo(value, min);
    }

    /**
     * Gets the distance to the maximum
     * @param value the value to check
     * @return the distance to the maximum
     */
    public double distanceToMax(double value) {
        return distanceTo(value, max);
    }

    /**
     * Gets the minimum value
     * @return the minimum value
     */
    public double getMin(){
        return min;
    }
    /**
     * Gets the maximum value
     * @return the maximum value
     */
    public double getMax(){
        return max;
    }

    /** Returns true if the value is strictly between min and max (ignores edges). */
    public boolean isStrictlyInside(double value) {
        return value > min && value < max;
    }

    /**
     * Check if a value is in the interval
     * @param value the value to check
     * @return true if the value is in the domain
     */
    public boolean isInRange(double value){

        boolean minBool;
        boolean maxBool;

        if (minEdge == IntervalEdge.EMPTY) {
            minBool = value > min;
        }else{
            minBool = value >= min;
        }

        if (maxEdge == IntervalEdge.EMPTY) {
            maxBool = value < max;
        }else{
            maxBool = value <= max;
        }

        return  minBool && maxBool;
    }

    /**
     * Expands the interval by a fixed amount on each side.
     * @param amount the amount to expand the interval by
     */
    public void expand(double amount) {
        min -= amount;
        max += amount;
    }

    /**
     * Gets the length of the interval
     * @return the length of the interval
     */
    public double length() {
        return max - min;
    }

    /**
     * Gets the midpoint of the interval
     * @return the midpoint of the interval
     */
    public double midpoint() {
        return min + (max - min) / 2.0;
    }

    /**
     * Expands the interval by a percentage of its current size on each side.
     * @param percent the percentage to expand the interval by (e.g., 0.1 for 10%)
     */
    public void expandPercent(double percent) {
        double delta = (max - min) * percent;
        expand(delta);
    }

    /**
     * Checks if this interval overlaps with another interval
     * @param other the other interval
     * @return true if the intervals overlap
     */
    public boolean overlaps(Interval other) {
        return this.max >= other.min && other.max >= this.min;
    }

    /**
     * Checks if a value is in the max of the interval
     * @param value the value to check
     * @return true if the value is at maximum
     */
    public boolean isAtMax(double value){
        boolean maxBool;

        if (maxEdge == IntervalEdge.EMPTY) {
            maxBool = value < max;
        }else{
            maxBool = value <= max;
        }

        return maxBool;
    }

    /**
     * Checks if a value is in the min of the interval
     * @param value the value to check
     * @return true if the value is at minimum
     */
    public boolean isAtMin(double value){
        boolean minBool;

        if (minEdge == IntervalEdge.EMPTY) {
            minBool = value > min;
        }else{
            minBool = value >= min;
        }

        return minBool;
    }

    /**
     * Check if a value is out of the interval
     * @param value the value to check
     * @return true if the value is out of the interval
     */
    public boolean isOutOfBounds(double value){
        return !isInRange(value);
    }

}