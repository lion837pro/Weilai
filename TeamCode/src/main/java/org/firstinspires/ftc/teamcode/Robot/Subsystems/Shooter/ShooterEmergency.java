package org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

/**
 * EMERGENCY SHOOTER SUBSYSTEM
 * Uses standard REV DcMotor instead of MotorEx.
 * Simple power-based control without complex PID velocity control.
 * For use when the normal shooter setup doesn't work.
 */
public class ShooterEmergency implements Subsystem {
    public static final ShooterEmergency INSTANCE = new ShooterEmergency();

    // Standard REV DcMotor
    private DcMotor motor;

    // State
    private double currentPower = 0;
    private double targetPower = 0;

    // Encoder tracking for RPM display
    private int lastEncoderPosition = 0;
    private long lastEncoderTime = 0;
    private double currentVelocityTPS = 0;

    // Constants
    private static final double TICKS_PER_REV = 28.0;  // REV HD Hex motor
    private static final double RAMP_RATE = 0.05;  // Power change per update cycle

    private Command defaultCommand = new NullCommand();

    @Override
    public void initialize() {
        try {
            motor = ActiveOpMode.hardwareMap().get(DcMotor.class, ShooterConstants.SHOOTER_MOTOR_NAME);

            if (ShooterConstants.MOTOR_INVERTED) {
                motor.setDirection(DcMotorSimple.Direction.REVERSE);
            }

            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            lastEncoderPosition = motor.getCurrentPosition();
            lastEncoderTime = System.nanoTime();

            ActiveOpMode.telemetry().addData("Emergency Shooter", "READY");
        } catch (Exception e) {
            motor = null;
            ActiveOpMode.telemetry().addData("Emergency Shooter", "NOT FOUND: " + e.getMessage());
        }
    }

    @NonNull
    @Override
    public Command getDefaultCommand() {
        return defaultCommand;
    }

    public void setDefaultCommand(Command command) {
        this.defaultCommand = command;
    }

    @Override
    public void periodic() {
        // Calculate velocity for telemetry
        updateVelocity();

        // Ramp power smoothly to prevent belt slip
        if (currentPower < targetPower) {
            currentPower = Math.min(currentPower + RAMP_RATE, targetPower);
        } else if (currentPower > targetPower) {
            currentPower = Math.max(currentPower - RAMP_RATE, targetPower);
        }

        // Apply power
        if (motor != null) {
            motor.setPower(currentPower);
        }

        // Telemetry
        try {
            ActiveOpMode.telemetry().addData("--- EMERGENCY SHOOTER ---", "");
            ActiveOpMode.telemetry().addData("Target Power", "%.2f", targetPower);
            ActiveOpMode.telemetry().addData("Current Power", "%.2f", currentPower);
            ActiveOpMode.telemetry().addData("Current RPM", "%.0f", getCurrentRPM());
            ActiveOpMode.telemetry().addData("Motor", motor != null ? "OK" : "NOT FOUND");
        } catch (Exception e) {
            // Ignore telemetry errors
        }
    }

    private void updateVelocity() {
        if (motor == null) return;

        long currentTime = System.nanoTime();
        int currentPosition = motor.getCurrentPosition();

        double deltaTime = (currentTime - lastEncoderTime) / 1e9;  // Convert to seconds
        if (deltaTime > 0.01) {  // Update every 10ms minimum
            int deltaTicks = currentPosition - lastEncoderPosition;
            currentVelocityTPS = deltaTicks / deltaTime;

            lastEncoderPosition = currentPosition;
            lastEncoderTime = currentTime;
        }
    }

    // ===== PUBLIC CONTROL METHODS =====

    /**
     * Set motor power directly (0 to 1)
     */
    public void setPower(double power) {
        targetPower = Math.max(-1.0, Math.min(1.0, power));
    }

    /**
     * Set power based on target RPM (approximate)
     * Uses a simple linear mapping since we don't have PID
     */
    public void setRPM(double rpm) {
        // Linear approximation: power = rpm / maxRPM
        double power = rpm / ShooterConstants.MAX_RPM;
        setPower(power);
    }

    /**
     * Stop the shooter
     */
    public void stop() {
        targetPower = 0;
    }

    /**
     * Get current velocity in ticks per second
     */
    public double getVelocity() {
        return currentVelocityTPS;
    }

    /**
     * Get current RPM
     */
    public double getCurrentRPM() {
        return (currentVelocityTPS * 60.0) / TICKS_PER_REV;
    }

    /**
     * Check if shooter is at target power (within ramp tolerance)
     */
    public boolean atTargetPower() {
        return Math.abs(currentPower - targetPower) < 0.02;
    }

    /**
     * Check if shooter is spinning (RPM > threshold)
     */
    public boolean isSpinning() {
        return Math.abs(getCurrentRPM()) > 100;
    }

    /**
     * Check if shooter RPM is approximately at target
     */
    public boolean atApproximateRPM(double targetRPM, double tolerance) {
        return Math.abs(getCurrentRPM() - targetRPM) < tolerance;
    }

    public SubsystemComponent asCOMPONENT() {
        return new SubsystemComponent(INSTANCE);
    }
}
