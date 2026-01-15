package org.firstinspires.ftc.teamcode.Robot.Subsystems.Hood;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.SuperChassis;

import java.util.function.DoubleSupplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;

/**
 * Commands for the Hood subsystem.
 *
 * Provides hood angle control for distance-based aiming.
 */
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
}
