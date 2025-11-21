package org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Utils;

import java.util.function.DoubleSupplier;

/**
 * A class that contains methods for filtering the input of a {@link DoubleSupplier}.
 */
public class FilteredDoubleSupplier{

    /**
     * Filters the input of a {@link DoubleSupplier} using a kE value.
     *
     * @param input The input {@link DoubleSupplier} to filter.
     * @param kE The kE value to filter the input with.
     * @return A new {@link DoubleSupplier} that filters the input using the kE value.
     */
    public static DoubleSupplier apply(DoubleSupplier input, double kE){

        return ()-> Math.signum(input.getAsDouble()) * Math.pow(Math.abs(input.getAsDouble()), kE);
    }

    /**
     * Filters the input of a {@link DoubleSupplier} using a kE and kI value.
     *
     * @param input The input {@link DoubleSupplier} to filter.
     * @param kE The kE value to filter the input with.
     * @param kI The kI value to filter the input with.
     * @return A new {@link DoubleSupplier} that filters the input using the kE and kI values.
     */
    public static DoubleSupplier apply(DoubleSupplier input, double kE, double kI){
        return ()-> Math.signum(input.getAsDouble()) * Math.pow(Math.abs(input.getAsDouble()), kE) + kI;
    }

    /**
     * Filters the input of a {@link DoubleSupplier} using a kE, kI, and kD value.
     *
     * @param input The input {@link DoubleSupplier} to filter.
     * @param kE The kE value to filter the input with.
     * @param kI The kI value to filter the input with.
     * @param kD The kD value to filter the input with.
     * @return A new {@link DoubleSupplier} that filters the input using the kE, kI, and kD values.
     */
    public static DoubleSupplier apply(DoubleSupplier input, double kE, double kI, double kD){
        return ()-> Math.signum(input.getAsDouble()) *
                Math.pow(Math.abs(input.getAsDouble()), kE) + kI +
                Math.signum(input.getAsDouble()) *
                        Math.pow(Math.abs(input.getAsDouble()), kD);
    }

}


