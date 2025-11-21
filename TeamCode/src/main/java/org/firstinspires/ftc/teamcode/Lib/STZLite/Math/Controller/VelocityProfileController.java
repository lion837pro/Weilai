package org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Controller;

public class VelocityProfileController {
    private final double kP;
    private final double kS;
    private final double kV;

    private double targetVelocity = 0.0;

    public VelocityProfileController(double kP, double kS, double kV) {
        this.kP = kP;
        this.kS = kS;
        this.kV = kV;
    }

    public void setTarget(double velocity) {
        this.targetVelocity = velocity;
    }

    public double getTarget() {
        return this.targetVelocity;
    }


    public double calculate(double currentVelocity) {

        double error = targetVelocity - currentVelocity;

        double ff = kS + kV * targetVelocity;
        double p = kP * error;

        double output = ff + p;

        return Math.max(-1, Math.min(1, output));
    }

}
