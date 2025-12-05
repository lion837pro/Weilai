package org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Controller;

public class VelocityProfileController {
    private final double kP;
    private final double kI;
    private final double kD;
    private final double kS;
    private final double kV;

    private double targetVelocity = 0.0;
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private long lastTime = 0;

    // Legacy constructor for backward compatibility (kI=0, kD=0)
    public VelocityProfileController(double kP, double kS, double kV) {
        this(kP, 0.0, 0.0, kS, kV);
    }

    // Full PID constructor
    public VelocityProfileController(double kP, double kI, double kD, double kS, double kV) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;
        this.kV = kV;
    }

    public void setTarget(double velocity) {
        this.targetVelocity = velocity;
        // Reset integral when target changes to prevent windup
        this.integralSum = 0.0;
        this.lastError = 0.0;
    }

    public double getTarget() {
        return this.targetVelocity;
    }

    public double calculate(double currentVelocity) {
        long currentTime = System.nanoTime();
        double dt = lastTime == 0 ? 0.02 : (currentTime - lastTime) / 1e9; // Default to 20ms if first call
        lastTime = currentTime;

        double error = targetVelocity - currentVelocity;

        // Proportional term
        double p = kP * error;

        // Integral term (with anti-windup: clamp accumulation)
        integralSum += error * dt;
        integralSum = Math.max(-100, Math.min(100, integralSum)); // Prevent integral windup
        double i = kI * integralSum;

        // Derivative term
        double derivative = (error - lastError) / dt;
        double d = kD * derivative;
        lastError = error;

        // Feedforward term
        double ff = Math.signum(targetVelocity) * kS + kV * targetVelocity;

        // Combine all terms
        double output = ff + p + i + d;

        return Math.max(-1, Math.min(1, output));
    }

    // Reset for when subsystem is disabled/re-enabled
    public void reset() {
        integralSum = 0.0;
        lastError = 0.0;
        lastTime = 0;
    }
}
