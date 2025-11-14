package org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Utils;

/**
 * The {@code Epsilon} class provides utility methods to compare floating-point numbers
 * with a specified tolerance (epsilon).
 * This class contains static methods to check if two double values are equal within a given epsilon.
 */
public class Epsilon{

    /**
     * A small epsilon value used as the default tolerance for comparisons.
     */
    public static double kEpsilon = 1e-9;

    /**
     * Compares two {@code double} values for equality within a given epsilon.
     *
     * @param a The first value to compare.
     * @param b The second value to compare.
     * @param epsilon The tolerance within which the two values are considered equal.
     * @return {@code true} if the absolute difference between {@code a} and {@code b} is less than or equal to {@code epsilon}, otherwise {@code false}.
     */
    public static boolean equals(double a, double b, double epsilon){
        return(a - epsilon <= b) && (a + epsilon >= b);
    }

    /**
     * Compares two {@code double} values for equality using the default epsilon value.
     *
     * @param a The first value to compare.
     * @param b The second value to compare.
     * @return {@code true} if the absolute difference between {@code a} and {@code b} is less than or equal to the default epsilon value, otherwise {@code false}.
     */
    public static boolean equals(double a, double b){
        return equals(a, b, kEpsilon);
    }

}
