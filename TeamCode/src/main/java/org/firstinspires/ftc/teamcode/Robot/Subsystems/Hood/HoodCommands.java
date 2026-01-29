package org.firstinspires.ftc.teamcode.Robot.Subsystems.Hood;

/*
 * HOOD COMMANDS - COMMENTED OUT
 * Using RPM-based distance shooting instead of hood angle adjustment.
 * To re-enable, uncomment this entire file.
 */

/*
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.SuperChassis;

import java.util.function.DoubleSupplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;

public class HoodCommands {

    // ========================================================================
    // INSTANT COMMANDS
    // ========================================================================

    /**
     * Set hood to a specific angle (instant command)
     */
    public static Command setAngle(Hood hood, double degrees) {
        return new InstantCommand("setHoodAngle", () -> hood.setAngleDegrees(degrees));
    }

    /**
     * Set hood for close-range shots
     */
    public static Command setClose(Hood hood) {
        return new InstantCommand("setHoodClose", hood::setClose);
    }

    /**
     * Set hood for mid-range shots
     */
    public static Command setMid(Hood hood) {
        return new InstantCommand("setHoodMid", hood::setMid);
    }

    /**
     * Set hood for far-range shots
     */
    public static Command setFar(Hood hood) {
        return new InstantCommand("setHoodFar", hood::setFar);
    }

    /**
     * Set hood to default position
     */
    public static Command setDefault(Hood hood) {
        return new InstantCommand("setHoodDefault", hood::setDefault);
    }

    // ========================================================================
    // CONTINUOUS COMMANDS
    // ========================================================================

    /**
     * Continuously adjust hood based on vision distance.
     * Runs until interrupted - ideal for TeleOp auto-aim.
     */
    public static Command autoAimHood(Hood hood, SuperChassis chassis) {
        return new LambdaCommand()
                .named("autoAimHood")
                .requires(hood)
                .setStart(() -> {})
                .setUpdate(() -> {
                    double distance = chassis.getDistanceToTag();
                    if (distance > 0) {
                        hood.setHoodForDistance(distance);
                    }
                })
                .setStop(interrupted -> {})
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    /**
     * Auto-aim hood with distance supplier (for custom distance sources)
     */
    public static Command autoAimHood(Hood hood, DoubleSupplier distanceSupplier) {
        return new LambdaCommand()
                .named("autoAimHood")
                .requires(hood)
                .setStart(() -> {})
                .setUpdate(() -> {
                    double distance = distanceSupplier.getAsDouble();
                    if (distance > 0) {
                        hood.setHoodForDistance(distance);
                    }
                })
                .setStop(interrupted -> {})
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    /**
     * Manual hood adjustment with joystick/trigger input.
     * Input is scaled for fine control.
     */
    public static Command manualAdjust(Hood hood, DoubleSupplier input) {
        final double ADJUST_RATE = 30.0; // degrees per second at full input
        final long[] lastTime = {System.nanoTime()};

        return new LambdaCommand()
                .named("manualHoodAdjust")
                .requires(hood)
                .setStart(() -> lastTime[0] = System.nanoTime())
                .setUpdate(() -> {
                    double inputValue = input.getAsDouble();
                    if (Math.abs(inputValue) < 0.1) return;

                    long currentTime = System.nanoTime();
                    double dt = (currentTime - lastTime[0]) / 1e9;
                    lastTime[0] = currentTime;

                    double deltaAngle = inputValue * ADJUST_RATE * dt;
                    hood.adjustAngle(deltaAngle);
                })
                .setStop(interrupted -> {})
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    /**
     * Set hood for a specific distance (instant command)
     */
    public static Command setForDistance(Hood hood, double distanceInches) {
        return new InstantCommand("setHoodForDistance",
                () -> hood.setHoodForDistance(distanceInches));
    }

    // ========================================================================
    // ODOMETRY-BASED COMMANDS
    // ========================================================================

    /**
     * Auto-aim hood based on odometry distance to a fixed field position.
     * Uses robot pose from odometry to calculate distance.
     * Runs continuously until interrupted.
     *
     * @param hood The hood subsystem
     * @param chassis The chassis (provides robot pose from odometry)
     * @param targetX Target X position on field (inches)
     * @param targetY Target Y position on field (inches)
     */
    public static Command autoAimOdometry(Hood hood, SuperChassis chassis,
                                           double targetX, double targetY) {
        return new LambdaCommand()
                .named("HoodOdometryAim")
                .requires(hood)
                .setStart(() -> {})
                .setUpdate(() -> {
                    double distance = chassis.getDistanceToPosition(targetX, targetY);
                    if (distance > 0) {
                        hood.setHoodForDistance(distance);
                    }

                    dev.nextftc.ftc.ActiveOpMode.telemetry().addData("Hood Mode", "ODOMETRY");
                    dev.nextftc.ftc.ActiveOpMode.telemetry().addData("Distance", "%.1f in", distance);
                })
                .setStop(interrupted -> {})
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    // ========================================================================
    // HYBRID COMMANDS (LIMELIGHT + ODOMETRY FALLBACK)
    // ========================================================================

    /**
     * Hybrid auto-aim: Uses Limelight distance when target visible,
     * falls back to odometry-based distance when vision is unavailable.
     * Best of both worlds - precise vision distance when possible,
     * predictive odometry when vision is unavailable.
     *
     * @param hood The hood subsystem
     * @param chassis The chassis (provides Limelight data and odometry)
     * @param fallbackTargetX Field X position for odometry fallback (inches)
     * @param fallbackTargetY Field Y position for odometry fallback (inches)
     */
    public static Command hybridAutoAim(Hood hood, SuperChassis chassis,
                                         double fallbackTargetX, double fallbackTargetY) {
        final boolean[] usingVision = {false};

        return new LambdaCommand()
                .named("HoodHybridAim")
                .requires(hood)
                .setStart(() -> {
                    usingVision[0] = false;
                })
                .setUpdate(() -> {
                    // Check if Limelight has a valid target
                    boolean hasVisionTarget = chassis.hasValidDistanceTarget();

                    if (hasVisionTarget) {
                        // Use Limelight-based distance
                        double distance = chassis.getDistanceToTag();
                        if (distance > 0) {
                            hood.setHoodForDistance(distance);
                            usingVision[0] = true;

                            dev.nextftc.ftc.ActiveOpMode.telemetry().addData("Hood Mode", "VISION");
                            dev.nextftc.ftc.ActiveOpMode.telemetry().addData("LL Distance", "%.1f in", distance);
                        }
                    } else {
                        // Fallback to odometry-based distance
                        double distance = chassis.getDistanceToPosition(fallbackTargetX, fallbackTargetY);
                        if (distance > 0) {
                            hood.setHoodForDistance(distance);
                            usingVision[0] = false;

                            dev.nextftc.ftc.ActiveOpMode.telemetry().addData("Hood Mode", "ODOMETRY");
                            dev.nextftc.ftc.ActiveOpMode.telemetry().addData("Odom Distance", "%.1f in", distance);
                        }
                    }
                })
                .setStop(interrupted -> {})
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    /**
     * Hybrid auto-aim with manual override capability.
     * If joystick input is detected, switches to manual control.
     * Otherwise uses vision/odometry for automatic aiming.
     *
     * @param hood The hood subsystem
     * @param chassis The chassis
     * @param manualInput Joystick/trigger input for manual override
     * @param fallbackTargetX Field X position for odometry fallback (inches)
     * @param fallbackTargetY Field Y position for odometry fallback (inches)
     */
    public static Command hybridAutoAimWithManual(Hood hood, SuperChassis chassis,
                                                   DoubleSupplier manualInput,
                                                   double fallbackTargetX, double fallbackTargetY) {
        final double ADJUST_RATE = 30.0; // degrees per second at full input
        final long[] lastTime = {System.nanoTime()};

        return new LambdaCommand()
                .named("HoodHybridManual")
                .requires(hood)
                .setStart(() -> lastTime[0] = System.nanoTime())
                .setUpdate(() -> {
                    double inputValue = manualInput.getAsDouble();

                    // Check for manual override
                    if (Math.abs(inputValue) > 0.1) {
                        // Manual control mode
                        long currentTime = System.nanoTime();
                        double dt = (currentTime - lastTime[0]) / 1e9;
                        lastTime[0] = currentTime;

                        double deltaAngle = inputValue * ADJUST_RATE * dt;
                        hood.adjustAngle(deltaAngle);

                        dev.nextftc.ftc.ActiveOpMode.telemetry().addData("Hood Mode", "MANUAL");
                        return;
                    }

                    lastTime[0] = System.nanoTime();

                    // Auto-aim mode (vision or odometry)
                    if (chassis.hasValidDistanceTarget()) {
                        double distance = chassis.getDistanceToTag();
                        if (distance > 0) {
                            hood.setHoodForDistance(distance);
                            dev.nextftc.ftc.ActiveOpMode.telemetry().addData("Hood Mode", "VISION");
                        }
                    } else {
                        double distance = chassis.getDistanceToPosition(fallbackTargetX, fallbackTargetY);
                        if (distance > 0) {
                            hood.setHoodForDistance(distance);
                            dev.nextftc.ftc.ActiveOpMode.telemetry().addData("Hood Mode", "ODOMETRY");
                        }
                    }
                })
                .setStop(interrupted -> {})
                .setIsDone(() -> false)
                .setInterruptible(true);
    }
}
*/
