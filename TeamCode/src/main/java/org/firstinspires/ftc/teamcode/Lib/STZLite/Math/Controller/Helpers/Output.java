package org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Controller.Helpers;

@FunctionalInterface
public interface Output{
    double getAsDouble();

    default Output plus(Output other){
        return ()-> getAsDouble() + other.getAsDouble();
    }

    default Output minus(Output other){
        return ()-> getAsDouble() - other.getAsDouble();
    }

    default Output clamp(double min, double max){
        return ()-> Math.max(Math.min(getAsDouble(), max), min);
    }

    default Output negate(){
        return ()-> -getAsDouble();
    }

    default Output times(double scalar){
        return ()-> getAsDouble() * scalar;
    }

    default Output divide(double scalar){
        return ()-> getAsDouble() / scalar;
    }

    default Output withDeadband(double threshold) {
        return () -> Math.abs(getAsDouble()) > threshold ? getAsDouble() : 0.0;
    }

}
