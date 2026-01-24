package org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import androidx.annotation.NonNull;

// import com.qualcomm.robotcore.hardware.Servo;  // Hood disabled - using RPM-based distance

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

import org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Controller.VelocityProfileController;
import org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Intervals.Interval;
import org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Controller.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret.Turret;

public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();

    private VelocityProfileController controller;
    private SlewRateLimiter slewRateLimiter;

    // Shooter motor (single motor)
    private MotorEx motor;

    // Hood servos - DISABLED (using RPM-based distance shooting)
    // private Servo hoodServo1;
    // private Servo hoodServo2;

    // State
    private boolean hasTarget = false;
    private boolean open = false;
    private double currentPower = 0;
    private double rawPower = 0;
    // private double currentHoodPosition = ShooterConstants.HOOD_DEFAULT_POSITION;  // Hood disabled

    // Power efficiency: track if motor is idle for float mode
    private boolean isIdle = true;

    private Command defaultCommand = new NullCommand();

    @Override
    public void initialize() {
        // Initialize motor
        try {
            this.motor = new MotorEx(ShooterConstants.SHOOTER_MOTOR_NAME);
            if (ShooterConstants.MOTOR_INVERTED) {
                motor.reversed();
            }
            motor.brakeMode();
        } catch (Exception e) {
            this.motor = null;
            ActiveOpMode.telemetry().addData("Shooter Motor", "NOT FOUND");
        }

        // Hood servos - DISABLED (using RPM-based distance shooting)
        /*
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
        */

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
            // Only allow revving if turret is clockwise or static (not counter-clockwise)
            if (Turret.INSTANCE.canShooterRev()) {
                rawPower = controller.calculate(getVelocity());

                // Apply slew rate limiting to prevent belt slip
                double limitedPower = slewRateLimiter.calculate(rawPower);

                setPower(limitedPower);
            } else {
                // Turret is turning counter-clockwise, don't rev
                setPower(0);
                rawPower = 0;
            }
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

            // F. Hood status - DISABLED (using RPM-based distance shooting)
            // ActiveOpMode.telemetry().addData("--- HOOD ---", "");
            // ActiveOpMode.telemetry().addData("Hood Position", "%.2f", currentHoodPosition);
            // ActiveOpMode.telemetry().addData("Servo1", hoodServo1 != null ? "OK" : "NOT FOUND");
            // ActiveOpMode.telemetry().addData("Servo2", hoodServo2 != null ? "OK" : "NOT FOUND");

            // G. Motor status
            ActiveOpMode.telemetry().addData("--- MOTOR ---", "");
            ActiveOpMode.telemetry().addData("Motor", motor != null ? "OK" : "NOT FOUND");

            // H. Turret-Shooter interlock status
            ActiveOpMode.telemetry().addData("--- TURRET LOCK ---", "");
            ActiveOpMode.telemetry().addData("Turret Allows Rev", Turret.INSTANCE.canShooterRev() ? "YES" : "NO (CCW)");

            // Don't call update() here - let the OpMode handle telemetry updates
        } catch (Exception e) {
            // Failsafe if telemetry isn't ready
        }
    }

    public double getVelocity(){
        if (motor == null) return 0;
        return motor.getVelocity();
    }

    public void toVelocity(double velocity){
        this.hasTarget = true;
        this.open = false;
        // Only update target if it changed to preserve integral accumulation
        if (Math.abs(controller.getTarget() - velocity) > 1.0) {
            controller.setTarget(velocity);
        }
    }

    private void setPower(double power) {
        this.currentPower = power;

        if (motor == null) return;

        // Power efficiency: switch to float mode when idle
        if (Math.abs(power) < 0.01) {
            if (!isIdle) {
                // Switch to float mode for power savings when stopping
                motor.floatMode();
                isIdle = true;
            }
            motor.setPower(0);
        } else {
            if (isIdle) {
                // Switch back to brake mode when running for better control
                motor.brakeMode();
                isIdle = false;
            }
            motor.setPower(power);
        }
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

    // ===== HOOD CONTROL - DISABLED (using RPM-based distance shooting) =====
    /*
    public void setHoodPosition(double position) {
        position = Math.max(ShooterConstants.HOOD_MIN_POSITION,
                Math.min(ShooterConstants.HOOD_MAX_POSITION, position));
        this.currentHoodPosition = position;
        if (hoodServo1 != null) hoodServo1.setPosition(position);
        if (hoodServo2 != null) hoodServo2.setPosition(position);
    }

    public double getHoodPosition() { return currentHoodPosition; }
    public void setHoodClose() { setHoodPosition(ShooterConstants.HOOD_CLOSE_SHOT); }
    public void setHoodMid() { setHoodPosition(ShooterConstants.HOOD_MID_SHOT); }
    public void setHoodFar() { setHoodPosition(ShooterConstants.HOOD_FAR_SHOT); }
    public void adjustHood(double delta) { setHoodPosition(currentHoodPosition + delta); }

    public void setHoodForDistance(double distanceInches) {
        double minDist = 24.0;
        double maxDist = 72.0;
        if (distanceInches <= minDist) {
            setHoodPosition(ShooterConstants.HOOD_CLOSE_SHOT);
        } else if (distanceInches >= maxDist) {
            setHoodPosition(ShooterConstants.HOOD_FAR_SHOT);
        } else {
            double t = (distanceInches - minDist) / (maxDist - minDist);
            double position = ShooterConstants.HOOD_CLOSE_SHOT +
                    t * (ShooterConstants.HOOD_FAR_SHOT - ShooterConstants.HOOD_CLOSE_SHOT);
            setHoodPosition(position);
        }
    }
    */

    public SubsystemComponent asCOMPONENT(){return new SubsystemComponent(INSTANCE);}

}
