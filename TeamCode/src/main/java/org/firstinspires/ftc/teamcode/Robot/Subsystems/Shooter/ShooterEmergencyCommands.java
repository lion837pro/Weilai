package org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.LED.RobotFeedback;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer.Spindexer;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer.SpindexerCommands;

import java.util.function.DoubleSupplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.commands.utility.SequentialGroup;
import dev.nextftc.core.commands.utility.WaitCommand;

/**
 * Emergency Shooter Commands
 * Simple power-based control for the emergency shooter subsystem.
 */
public class ShooterEmergencyCommands {

    // ===== POWER PRESETS =====
    // These are approximate power levels for different shot distances
    public static final double POWER_LOW = 0.4;      // ~1400 RPM - close shots
    public static final double POWER_MEDIUM = 0.5;   // ~1800 RPM - medium shots
    public static final double POWER_HIGH = 0.6;     // ~2200 RPM - far shots
    public static final double POWER_MAX = 0.75;     // ~2800 RPM - maximum

    // ===== INSTANT COMMANDS =====

    /**
     * Set shooter to specific power
     */
    public static Command setPower(ShooterEmergency shooter, double power) {
        return new InstantCommand("SetShooterPower", () -> shooter.setPower(power));
    }

    /**
     * Stop shooter
     */
    public static Command stop(ShooterEmergency shooter) {
        return new InstantCommand("StopShooter", shooter::stop);
    }

    // ===== CONTINUOUS COMMANDS =====

    /**
     * Run shooter at fixed power continuously
     */
    public static Command runAtPower(ShooterEmergency shooter, double power) {
        return new LambdaCommand()
                .named("RunShooterAtPower")
                .requires(shooter)
                .setStart(() -> shooter.setPower(power))
                .setUpdate(() -> shooter.setPower(power))
                .setStop(interrupted -> shooter.stop())
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    /**
     * Run shooter at fixed power with feedback when ready
     */
    public static Command runAtPower(ShooterEmergency shooter, double power, RobotFeedback feedback) {
        final boolean[] notifiedReady = {false};
        final double targetRPM = power * ShooterConstants.MAX_RPM;

        return new LambdaCommand()
                .named("RunShooterAtPower")
                .requires(shooter)
                .setStart(() -> {
                    shooter.setPower(power);
                    notifiedReady[0] = false;
                })
                .setUpdate(() -> {
                    shooter.setPower(power);
                    // Notify when approximately at target
                    if (!notifiedReady[0] && shooter.atTargetPower() && shooter.isSpinning()) {
                        if (feedback != null) {
                            feedback.onShooterAtRPM();
                        }
                        notifiedReady[0] = true;
                    }
                })
                .setStop(interrupted -> {
                    shooter.stop();
                    if (feedback != null) {
                        feedback.onShooterStop();
                    }
                })
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    /**
     * Manual power control with joystick/trigger
     */
    public static Command manualControl(ShooterEmergency shooter, DoubleSupplier input) {
        return new LambdaCommand()
                .named("ManualShooter")
                .requires(shooter)
                .setStart(() -> {})
                .setUpdate(() -> {
                    double power = input.getAsDouble();
                    if (Math.abs(power) > 0.05) {
                        shooter.setPower(power);
                    } else {
                        shooter.stop();
                    }
                })
                .setStop(interrupted -> shooter.stop())
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    // ===== SHOOTING SEQUENCES (NO INTAKE) =====

    /**
     * Spin up, feed balls, then stop.
     * For emergency no-intake mode.
     */
    public static Command shootNoIntake(ShooterEmergency shooter, Spindexer spindexer,
                                        double power, RobotFeedback feedback) {
        return new LambdaCommand()
                .named("EmergencyShoot")
                .requires(shooter)
                .setStart(() -> {
                    shooter.setPower(power);
                })
                .setUpdate(() -> {
                    shooter.setPower(power);
                })
                .setStop(interrupted -> {
                    shooter.stop();
                    if (feedback != null) {
                        feedback.onShooterStop();
                    }
                })
                .setIsDone(() -> false)
                .setInterruptible(true)
                .alongside(SpindexerCommands.smartFeedNoIntakeContinuous(shooter, spindexer, feedback));
    }

    /**
     * Simple shooting sequence with fixed spin-up time
     */
    public static Command shootWithSpinup(ShooterEmergency shooter, Spindexer spindexer,
                                          double power, int spinupMs, RobotFeedback feedback) {
        return new SequentialGroup(
                // Spin up
                new LambdaCommand()
                        .named("SpinUp")
                        .requires(shooter)
                        .setStart(() -> shooter.setPower(power))
                        .setUpdate(() -> {})
                        .setStop(interrupted -> {})
                        .setIsDone(() -> false)
                        .setInterruptible(true)
                        .withTimeout(spinupMs),

                // Notify ready
                new InstantCommand("NotifyReady", () -> {
                    if (feedback != null) {
                        feedback.onShooterAtRPM();
                    }
                }),

                // Feed and shoot continuously
                shootNoIntake(shooter, spindexer, power, feedback)
        );
    }

    // ===== PRESET COMMANDS =====

    /**
     * Low power shot (close range)
     */
    public static Command shootLow(ShooterEmergency shooter, Spindexer spindexer, RobotFeedback feedback) {
        return shootNoIntake(shooter, spindexer, POWER_LOW, feedback);
    }

    /**
     * Medium power shot
     */
    public static Command shootMedium(ShooterEmergency shooter, Spindexer spindexer, RobotFeedback feedback) {
        return shootNoIntake(shooter, spindexer, POWER_MEDIUM, feedback);
    }

    /**
     * High power shot (far range)
     */
    public static Command shootHigh(ShooterEmergency shooter, Spindexer spindexer, RobotFeedback feedback) {
        return shootNoIntake(shooter, spindexer, POWER_HIGH, feedback);
    }
}
