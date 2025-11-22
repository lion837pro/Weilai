package org.firstinspires.ftc.teamcode.Robot.Robott.Commands2;


import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.VisionConstants;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.Robot.Robott.Subsystems2.Vision;

import java.util.function.DoubleSupplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;

public class ShooterCommands2 {

    // ========== HELPER COMMANDS ==========

    /**
     * Smart feeder - only feeds when shooter is at target velocity
     */
    public static Command smartFeed(Shooter shooter, Intake intake) {
        return new LambdaCommand()
                .named("SmartFeed")
                .requires(intake)
                .setUpdate(() -> {
                    if (shooter.atSetpoint()) {
                        intake.MoveIn(1.0); // Fire!
                    } else {
                        intake.MoveIn(0);   // Wait for spin-up
                    }
                })
                .setStop(interrupted -> intake.MoveIn(0))
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    // ========== LOW-LEVEL COMMANDS ==========

    /**
     * Run shooter at manual power (open loop)
     */
    public static Command runManualShooter(Shooter shooter, DoubleSupplier powerSource) {
        return new LambdaCommand()
                .named("runManualShooter")
                .requires(shooter)
                .setStart(() -> {})
                .setUpdate(() -> {
                    double power = powerSource.getAsDouble();
                    if (Math.abs(power) < 0.1) power = 0;
                    shooter.set(power);
                })
                .setStop(interrupted -> shooter.set(0))
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    /**
     * Run shooter at a specific RPM using PID control
     */
    public static Command runShooterPID(Shooter shooter, double rpm) {
        double targetTPS = ShooterConstants.rpmToTicksPerSecond(rpm);
        return new LambdaCommand()
                .named("runShooterPID")
                .requires(shooter)
                .setStart(() -> shooter.toVelocity(targetTPS))
                .setUpdate(() -> shooter.toVelocity(targetTPS))
                .setStop(interrupted -> shooter.stop())
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    /**
     * Stop the shooter
     */
    public static Command stopShooter(Shooter shooter) {
        return new LambdaCommand()
                .named("stopShooter")
                .requires(shooter)
                .setStart(() -> shooter.stop())
                .setUpdate(() -> shooter.stop())
                .setIsDone(() -> true)
                .setInterruptible(true);
    }

    // ========== VISION-BASED AUTO COMMANDS ==========

    /**
     * Automatically calculate and set shooter RPM based on distance to AprilTag
     */
    public static Command autoRevShooter(Shooter shooter, Vision vision) {
        return new LambdaCommand()
                .named("autoRevShooter")
                .requires(shooter)
                .setUpdate(() -> {
                    // Get distance from vision
                    double distance = vision.getDistanceToTag();

                    // Calculate target RPM based on distance
                    double targetRPM = VisionConstants.BASE_RPM + (distance * VisionConstants.RPM_PER_INCH);

                    // Safety clamp
                    if (targetRPM > ShooterConstants.MAX_RPM) {
                        targetRPM = ShooterConstants.MAX_RPM;
                    }
                    if (distance <= 0 || !vision.hasTarget()) {
                        targetRPM = VisionConstants.BASE_RPM; // Default if no tag
                    }

                    // Convert and set
                    double targetTPS = ShooterConstants.rpmToTicksPerSecond(targetRPM);
                    shooter.toVelocity(targetTPS);

                    // Debug telemetry
                    dev.nextftc.ftc.ActiveOpMode.telemetry().addData("AutoAim Distance", "%.1f in", distance);
                    dev.nextftc.ftc.ActiveOpMode.telemetry().addData("AutoAim RPM", "%.0f", targetRPM);
                })
                .setStop(interrupted -> shooter.stop())
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    // ========== COMBO COMMANDS ==========

    /**
     * Shoot at a fixed RPM with smart feeding
     */
    public static Command shootWithFeed(Shooter shooter, Intake intake, double rpm) {
        return new ParallelGroup(
                runShooterPID(shooter, rpm),
                smartFeed(shooter, intake)
        );
    }

    /**
     * Auto-aim and shoot with AprilTag distance calculation
     */
    public static Command shootWithAutoAim(Shooter shooter, Intake intake, Vision vision) {
        return new ParallelGroup(
                autoRevShooter(shooter, vision),
                smartFeed(shooter, intake)
        );
    }
}