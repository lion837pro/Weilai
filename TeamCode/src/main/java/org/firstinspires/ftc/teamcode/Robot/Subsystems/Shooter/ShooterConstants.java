package org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

public class ShooterConstants {
    public static final String shootername1 = "Sh1";
    public static final String shootername2 = "Sh2";

    public static final double TICKS_PER_REV = 28.0; // For GoBilda Yellow Jacket 1:1
    public static final double MAX_RPM = 6000.0;
    public static double rpmToTicksPerSecond(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }
    public static double ticksPerSecondToRPM(double tps) {
        return (tps * 60.0) / TICKS_PER_REV;
    }
    public  static  final  boolean shootername2inverted = false;
    public  static  final  boolean shootername1inverted = true;

    public static final double kP = 0.15;
    public static final double kS = 0.00207;
    public static final double kV = 0.0005;

    public static final double velocityTolerance = 200;

}