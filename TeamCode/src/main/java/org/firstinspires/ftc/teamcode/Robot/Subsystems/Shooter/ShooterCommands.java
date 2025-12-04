package org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.SuperChassis;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.VisionConstants;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer.Spindexer;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer.SpindexerCommands;

import java.util.function.DoubleSupplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;

public class ShooterCommands {

    // ========================================================================
    // LOW-LEVEL SHOOTER COMMANDS
    // ========================================================================

    /**
     * Manual shooter control with joystick/trigger
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
     * Runs the shooter at a specific velocity using PID control.
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

    /**
     * Auto-rev shooter based on vision distance
     */
    public static Command autoRevShooter(Shooter shooter, SuperChassis chassis) {
        return new LambdaCommand()
                .named("autoRevShooter")
                .requires(shooter)
                .setUpdate(() -> {
                    double distance = chassis.getDistanceToTag();
                    double targetRPM = VisionConstants.BASE_RPM + (distance * VisionConstants.RPM_PER_INCH);

                    if (targetRPM > ShooterConstants.MAX_RPM) targetRPM = ShooterConstants.MAX_RPM;
                    if (distance <= 0) targetRPM = 1300;

                    double targetTPS = ShooterConstants.rpmToTicksPerSecond(targetRPM);
                    shooter.toVelocity(targetTPS);

                    dev.nextftc.ftc.ActiveOpMode.telemetry().addData("AutoAim Dist", "%.1f in", distance);
                    dev.nextftc.ftc.ActiveOpMode.telemetry().addData("AutoAim RPM", "%.0f", targetRPM);
                })
                .setStop(interrupted -> shooter.stop())
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    // ========================================================================
    // SHOOTING SEQUENCES WITH SPINDEXER
    // ========================================================================

    /**
     * Full shooting routine at fixed RPM with spindexer.
     * Shoots until all balls are fired.
     */
    public static Command shootAllBallsFixedRPM(Shooter shooter, Spindexer spindexer, Intake intake, double rpm) {
        return new ParallelGroup(
                runShooterPID(shooter, rpm),
                SpindexerCommands.smartFeedWithSpindexer(shooter, spindexer, intake)
        );
    }

    /**
     * Full shooting routine with auto-aim and spindexer.
     * Shoots until all balls are fired.
     */
    public static Command shootAllBallsAutoAim(Shooter shooter, Spindexer spindexer, Intake intake, SuperChassis chassis) {
        return new ParallelGroup(
                autoRevShooter(shooter, chassis),
                SpindexerCommands.smartFeedWithSpindexer(shooter, spindexer, intake)
        );
    }

    /**
     * TeleOp shooting with auto-aim - runs until button released.
     */
    public static Command teleopShootAutoAim(Shooter shooter, Spindexer spindexer, Intake intake, SuperChassis chassis) {
        return new ParallelGroup(
                autoRevShooter(shooter, chassis),
                SpindexerCommands.smartFeedWithSpindexerContinuous(shooter, spindexer, intake)
        );
    }

    /**
     * TeleOp shooting at fixed RPM - runs until button released.
     */
    public static Command teleopShootFixedRPM(Shooter shooter, Spindexer spindexer, Intake intake, double rpm) {
        return new ParallelGroup(
                runShooterPID(shooter, rpm),
                SpindexerCommands.smartFeedWithSpindexerContinuous(shooter, spindexer, intake)
        );
    }

    // ========================================================================
    // COLOR-SORTED SHOOTING SEQUENCES (uses AprilTag IDs 21, 22, 23)
    // ========================================================================

    /**
     * Shoot all balls with color sorting at fixed RPM.
     * Uses AprilTag IDs 21, 22, 23 to determine shooting order:
     * - Tag 21: Green first (slot 0)
     * - Tag 22: Green first (slot 1)
     * - Tag 23: Green first (slot 2)
     */
    public static Command shootAllBallsColorSorted(Shooter shooter, Spindexer spindexer,
                                                    Intake intake, SuperChassis chassis, double rpm) {
        return new ParallelGroup(
                runShooterPID(shooter, rpm),
                SpindexerCommands.smartFeedColorSorted(shooter, spindexer, intake, chassis)
        );
    }

    /**
     * Shoot all balls with color sorting and auto-aim.
     * Combines distance-based RPM with AprilTag color sorting.
     */
    public static Command shootAllBallsColorSortedAutoAim(Shooter shooter, Spindexer spindexer,
                                                           Intake intake, SuperChassis chassis) {
        return new ParallelGroup(
                autoRevShooter(shooter, chassis),
                SpindexerCommands.smartFeedColorSorted(shooter, spindexer, intake, chassis)
        );
    }

    /**
     * TeleOp color-sorted shooting at fixed RPM.
     * Continuously monitors AprilTag changes and adjusts shooting order.
     */
    public static Command teleopShootColorSorted(Shooter shooter, Spindexer spindexer,
                                                  Intake intake, SuperChassis chassis, double rpm) {
        return new ParallelGroup(
                runShooterPID(shooter, rpm),
                SpindexerCommands.smartFeedColorSortedContinuous(shooter, spindexer, intake, chassis)
        );
    }

    /**
     * TeleOp color-sorted shooting with auto-aim.
     * Best option for competition - combines distance-based RPM with color sorting.
     */
    public static Command teleopShootColorSortedAutoAim(Shooter shooter, Spindexer spindexer,
                                                         Intake intake, SuperChassis chassis) {
        return new ParallelGroup(
                autoRevShooter(shooter, chassis),
                SpindexerCommands.smartFeedColorSortedContinuous(shooter, spindexer, intake, chassis)
        );
    }
}
