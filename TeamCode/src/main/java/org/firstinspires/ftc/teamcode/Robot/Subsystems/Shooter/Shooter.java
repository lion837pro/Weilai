package org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Servo;

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

    // Shooter motors (2 motors)
    private MotorEx motor1;
    private MotorEx motor2;

    // Hood servos (2 servos for angle adjustment)
    private Servo hoodServo1;
    private Servo hoodServo2;

    // State
    private boolean hasTarget = false;
    private boolean open = false;
    private double currentPower = 0;
    private double rawPower = 0;
    private double currentHoodPosition = ShooterConstants.HOOD_DEFAULT_POSITION;

    private Command defaultCommand = new NullCommand();

    @Override
    public void initialize() {
        // Initialize motor 1
        try {
            this.motor1 = new MotorEx(ShooterConstants.SHOOTER_MOTOR_1_NAME);
            if (ShooterConstants.MOTOR_1_INVERTED) {
                motor1.reversed();
            }
            motor1.brakeMode();
        } catch (Exception e) {
            this.motor1 = null;
            ActiveOpMode.telemetry().addData("Shooter Motor 1", "NOT FOUND");
        }

        // Initialize motor 2
        try {
            this.motor2 = new MotorEx(ShooterConstants.SHOOTER_MOTOR_2_NAME);
            if (ShooterConstants.MOTOR_2_INVERTED) {
                motor2.reversed();
            }
            motor2.brakeMode();
        } catch (Exception e) {
            this.motor2 = null;
            ActiveOpMode.telemetry().addData("Shooter Motor 2", "NOT FOUND");
        }

        // Initialize hood servo 1
        try {
            this.hoodServo1 = ActiveOpMode.hardwareMap().get(Servo.class, ShooterConstants.HOOD_SERVO_1_NAME);
            if (ShooterConstants.HOOD_SERVO_1_REVERSED) {
                hoodServo1.setDirection(Servo.Direction.REVERSE);
            }
            hoodServo1.setPosition(ShooterConstants.HOOD_DEFAULT_POSITION);
        } catch (Exception e) {
            this.hoodServo1 = null;
            ActiveOpMode.telemetry().addData("Hood Servo 1", "NOT FOUND");
        }

        // Initialize hood servo 2
        try {
            this.hoodServo2 = ActiveOpMode.hardwareMap().get(Servo.class, ShooterConstants.HOOD_SERVO_2_NAME);
            if (ShooterConstants.HOOD_SERVO_2_REVERSED) {
                hoodServo2.setDirection(Servo.Direction.REVERSE);
            }
            hoodServo2.setPosition(ShooterConstants.HOOD_DEFAULT_POSITION);
        } catch (Exception e) {
            this.hoodServo2 = null;
            ActiveOpMode.telemetry().addData("Hood Servo 2", "NOT FOUND");
        }

        // Initialize slew rate limiter
        this.slewRateLimiter = new SlewRateLimiter(ShooterConstants.MAX_ACCELERATION);

        // Initialize controller with full PIDF gains
        if (ShooterConstants.USE_ADAPTIVE_KV) {
            this.controller = new VelocityProfileController(
                    ShooterConstants.kP,
                    ShooterConstants.kI,
                    ShooterConstants.kD,
                    ShooterConstants.kS,
                    ShooterConstants.TRANSITION_SPEED
            );
        } else {
            this.controller = new VelocityProfileController(
                    ShooterConstants.kP,
                    ShooterConstants.kI,
                    ShooterConstants.kD,
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

            // F. Hood status
            ActiveOpMode.telemetry().addData("--- HOOD ---", "");
            ActiveOpMode.telemetry().addData("Hood Position", "%.2f", currentHoodPosition);
            ActiveOpMode.telemetry().addData("Servo1", hoodServo1 != null ? "OK" : "NOT FOUND");
            ActiveOpMode.telemetry().addData("Servo2", hoodServo2 != null ? "OK" : "NOT FOUND");

            // G. Motor status
            ActiveOpMode.telemetry().addData("--- MOTORS ---", "");
            ActiveOpMode.telemetry().addData("Motor1", motor1 != null ? "OK" : "NOT FOUND");
            ActiveOpMode.telemetry().addData("Motor2", motor2 != null ? "OK" : "NOT FOUND");

            // Don't call update() here - let the OpMode handle telemetry updates
        } catch (Exception e) {
            // Failsafe if telemetry isn't ready
        }
    }

    public double getVelocity(){
        // Average velocity from both motors
        double vel1 = (motor1 != null) ? motor1.getVelocity() : 0;
        double vel2 = (motor2 != null) ? motor2.getVelocity() : 0;

        // If only one motor exists, return that velocity
        if (motor1 == null && motor2 == null) return 0;
        if (motor1 == null) return vel2;
        if (motor2 == null) return vel1;

        // Return average of both motors
        return (vel1 + vel2) / 2.0;
    }

    public void toVelocity(double velocity){
        this.hasTarget = true;
        this.open = false;
        controller.setTarget(velocity);
    }

    private void setPower(double power) {
        this.currentPower = power;
        if (motor1 != null) motor1.setPower(power);
        if (motor2 != null) motor2.setPower(power);
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

    // ===== HOOD CONTROL =====

    /**
     * Set hood position (0.0 to 1.0)
     * 0.0 = lowest angle (flat)
     * 1.0 = highest angle (steep)
     */
    public void setHoodPosition(double position) {
        position = Math.max(ShooterConstants.HOOD_MIN_POSITION,
                Math.min(ShooterConstants.HOOD_MAX_POSITION, position));
        this.currentHoodPosition = position;

        if (hoodServo1 != null) hoodServo1.setPosition(position);
        if (hoodServo2 != null) hoodServo2.setPosition(position);
    }

    /**
     * Get current hood position
     */
    public double getHoodPosition() {
        return currentHoodPosition;
    }

    /**
     * Set hood to close shot angle (low trajectory)
     */
    public void setHoodClose() {
        setHoodPosition(ShooterConstants.HOOD_CLOSE_SHOT);
    }

    /**
     * Set hood to mid shot angle (medium trajectory)
     */
    public void setHoodMid() {
        setHoodPosition(ShooterConstants.HOOD_MID_SHOT);
    }

    /**
     * Set hood to far shot angle (high trajectory)
     */
    public void setHoodFar() {
        setHoodPosition(ShooterConstants.HOOD_FAR_SHOT);
    }

    /**
     * Adjust hood position by a delta amount
     */
    public void adjustHood(double delta) {
        setHoodPosition(currentHoodPosition + delta);
    }

    /**
     * Calculate optimal hood position for a given distance (auto-aim)
     * This is a simple linear interpolation - tune based on testing
     */
    public void setHoodForDistance(double distanceInches) {
        // Map distance to hood position
        // Close (0-24"): HOOD_CLOSE_SHOT
        // Far (72"+): HOOD_FAR_SHOT
        // Linear interpolation in between

        double minDist = 24.0;
        double maxDist = 72.0;

        if (distanceInches <= minDist) {
            setHoodPosition(ShooterConstants.HOOD_CLOSE_SHOT);
        } else if (distanceInches >= maxDist) {
            setHoodPosition(ShooterConstants.HOOD_FAR_SHOT);
        } else {
            // Linear interpolation
            double t = (distanceInches - minDist) / (maxDist - minDist);
            double position = ShooterConstants.HOOD_CLOSE_SHOT +
                    t * (ShooterConstants.HOOD_FAR_SHOT - ShooterConstants.HOOD_CLOSE_SHOT);
            setHoodPosition(position);
        }
    }

    public SubsystemComponent asCOMPONENT(){return new SubsystemComponent(INSTANCE);}

}
