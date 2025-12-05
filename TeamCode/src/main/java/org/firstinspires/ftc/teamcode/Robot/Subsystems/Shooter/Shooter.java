package org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import androidx.annotation.NonNull;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

import org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Controller.VelocityProfileController;
import org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Intervals.Interval;
import org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Controller.SlewRateLimiter;

public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();

    private VelocityProfileController controller;
    private SlewRateLimiter slewRateLimiter;
     MotorEx Sh1;
    // MotorEx Sh2 ;
    private boolean hasTarget = false;
    private boolean open = false;
    private double currentPower = 0;
    private double rawPower = 0;

    private Command defaultCommand = new NullCommand();

    @Override
    public void initialize() {
        this.controller = new VelocityProfileController(
                ShooterConstants.kP,
                ShooterConstants.kS,
                ShooterConstants.kV);

        this.Sh1 = new MotorEx(ShooterConstants.shootername1).reversed();
       // this.Sh2 = new MotorEx(ShooterConstants.shootername2);

        Sh1.brakeMode();
      //  Sh2.brakeMode();
        // Initialize slew rate limiter
        // maxRateOfChange = how fast power can change per second
        // Example: 1.5 means 0 to 1.0 power takes ~0.67 seconds
        this.slewRateLimiter = new SlewRateLimiter(ShooterConstants.MAX_ACCELERATION);

        // Initialize controller (adaptive or simple based on constants)
        if (ShooterConstants.USE_ADAPTIVE_KV) {
            this.controller = new VelocityProfileController(
                    ShooterConstants.kP,
                    ShooterConstants.kS,
                    ShooterConstants.TRANSITION_SPEED
            );
        } else {
            this.controller = new VelocityProfileController(
                    ShooterConstants.kP,
                    ShooterConstants.kS,
                    ShooterConstants.kV
            );
        }
    }

    @NonNull
    @Override
    public Command getDefaultCommand() {
        return defaultCommand;
    }
    public void setDefaultCommand(Command command){
        this.defaultCommand = command;
    }


    @Override
    public void periodic() {

        if (hasTarget && !open) {
            rawPower = controller.calculate(getVelocity());

            // Apply slew rate limiting to prevent belt slip
            double limitedPower = slewRateLimiter.calculate(rawPower);

            setPower(limitedPower);
        }
        try {
            ActiveOpMode.telemetry().addData("--- SHOOTER DEBUG ---", "");

            // A. State
            ActiveOpMode.telemetry().addData("Mode", hasTarget ? "PID (Auto)" : "Manual (Power)");
            ActiveOpMode.telemetry().addData("Target Velocity (tps)", hasTarget ? controller.getTarget() : 0);
            ActiveOpMode.telemetry().addData("Target RPM", hasTarget ?
                    ShooterConstants.ticksPerSecondToRPM(controller.getTarget()) : 0);

            // B. Current State
            double currentTPS = getVelocity();
            double currentRPM = ShooterConstants.ticksPerSecondToRPM(currentTPS);
            ActiveOpMode.telemetry().addData("Current Velocity (tps)", "%.0f", currentTPS);
            ActiveOpMode.telemetry().addData("Current RPM", "%.0f", currentRPM);

            // C. Error
            if (hasTarget) {
                double errorTPS = controller.getTarget() - currentTPS;
                double errorRPM = ShooterConstants.ticksPerSecondToRPM(errorTPS);
                ActiveOpMode.telemetry().addData("Error (tps)", "%.0f", errorTPS);
                ActiveOpMode.telemetry().addData("Error (RPM)", "%.0f", errorRPM);
                ActiveOpMode.telemetry().addData("At Setpoint?", atSetpoint() ? "YES ✓" : "NO");
            }

            // D. Power Output (showing both raw and limited)
            ActiveOpMode.telemetry().addData("Raw Power (PID)", "%.3f", rawPower);
            ActiveOpMode.telemetry().addData("Limited Power (Actual)", "%.3f", currentPower);
            ActiveOpMode.telemetry().addData("Slew Active?", Math.abs(rawPower - currentPower) > 0.01 ? "YES" : "NO");

            // E. Constants (for verification)
            ActiveOpMode.telemetry().addData("Max Accel", "%.2f /sec", ShooterConstants.MAX_ACCELERATION);
            ActiveOpMode.telemetry().addData("Constants", "kP=%.5f, kS=%.5f, kV=%.5f",
                    ShooterConstants.kP, ShooterConstants.kS, ShooterConstants.kV);

            if (ShooterConstants.USE_ADAPTIVE_KV) {
                ActiveOpMode.telemetry().addData("Adaptive Mode", "Low=%.5f, High=%.5f",
                        ShooterConstants.LOW_SPEED_KV, ShooterConstants.HIGH_SPEED_KV);
            }
            // Don't call update() here - let the OpMode handle telemetry updates
        } catch (Exception e) {
            // Failsafe if telemetry isn't ready
        }
    }

    public double getVelocity(){
        // Only one motor active, so no averaging needed
        return Sh1.getVelocity();
    }

    public void toVelocity(double velocity){
        this.hasTarget = true;
        this.open = false;
        controller.setTarget(velocity);
    }

    private void setPower(double power) {
        this.currentPower = power;
        Sh1.setPower(power);
      //  Sh2.setPower(power);
    }

    public void set(double power) {
        hasTarget = false;
        open = true;
        setPower(power);
    }

    public void stop() {
        hasTarget = false;
        open = true;
        setPower(0);
    }

    public boolean atSetpoint() {
        if (!hasTarget) {
            return false;
        }

        double target = controller.getTarget();

        double minLimit = target - ShooterConstants.velocityTolerance;
        double maxLimit = target + ShooterConstants.velocityTolerance;

        return Interval.isInRange(getVelocity(), minLimit, maxLimit);
    }
    public SubsystemComponent asCOMPONENT(){return new SubsystemComponent(INSTANCE);}

}
