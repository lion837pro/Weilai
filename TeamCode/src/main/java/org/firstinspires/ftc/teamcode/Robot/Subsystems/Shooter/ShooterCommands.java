package org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.SuperChassis;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.VisionConstants;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Hood.Hood;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Hood.HoodConstants;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.LED.RobotFeedback;
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
     * Runs the shooter at a specific velocity with feedback.
     */
    public static Command runShooterPID(Shooter shooter, double rpm, RobotFeedback feedback) {
        double targetTPS = ShooterConstants.rpmToTicksPerSecond(rpm);
        final boolean[] hasNotifiedReady = {false};

        return new LambdaCommand()
                .named("runShooterPID")
                .requires(shooter)
                .setStart(() -> {
                    shooter.toVelocity(targetTPS);
                    hasNotifiedReady[0] = false;
                })
                .setUpdate(() -> {
                    shooter.toVelocity(targetTPS);

                    // Trigger feedback once when RPM is reached
                    if (shooter.atSetpoint() && !hasNotifiedReady[0]) {
                        if (feedback != null) {
                            feedback.onShooterAtRPM();
                        }
                        hasNotifiedReady[0] = true;
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

    // ========================================================================
    // DEPRECATED: Distance-based RPM auto-aim
    // Now using fixed high RPM (2500) with hood angle adjustment instead.
    // Use autoAimWithHood() for distance-based shooting.
    // ========================================================================

    /**
     * @deprecated Use autoAimWithHood() instead - fixed RPM with hood angle adjustment
     */
    @Deprecated
    public static Command autoRevShooter(Shooter shooter, SuperChassis chassis) {
        // Redirect to fixed RPM shooter at HoodConstants.FIXED_SHOOTING_RPM
        return runShooterPID(shooter, HoodConstants.FIXED_SHOOTING_RPM);
    }

    /**
     * @deprecated Use autoAimWithHood() instead - fixed RPM with hood angle adjustment
     */
    @Deprecated
    public static Command autoRevShooter(Shooter shooter, SuperChassis chassis, RobotFeedback feedback) {
        // Redirect to fixed RPM shooter at HoodConstants.FIXED_SHOOTING_RPM
        return runShooterPID(shooter, HoodConstants.FIXED_SHOOTING_RPM, feedback);
    }

    // ========================================================================
    // SHOOTING SEQUENCES WITH SPINDEXER
    // (Includes 60-degree mechanical offset for shooter spin-up clearance)
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
     * Full shooting routine at fixed RPM with feedback (LED + rumble)
     */
    public static Command shootAllBallsFixedRPM(Shooter shooter, Spindexer spindexer, Intake intake,
                                                 double rpm, RobotFeedback feedback) {
        return new ParallelGroup(
                runShooterPID(shooter, rpm),
                SpindexerCommands.smartFeedWithSpindexer(shooter, spindexer, intake, feedback)
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
     * Full shooting routine with auto-aim and feedback
     */
    public static Command shootAllBallsAutoAim(Shooter shooter, Spindexer spindexer, Intake intake,
                                                SuperChassis chassis, RobotFeedback feedback) {
        return new ParallelGroup(
                autoRevShooter(shooter, chassis, feedback),
                SpindexerCommands.smartFeedWithSpindexer(shooter, spindexer, intake, feedback)
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
     * TeleOp shooting with auto-aim and feedback
     */
    public static Command teleopShootAutoAim(Shooter shooter, Spindexer spindexer, Intake intake,
                                              SuperChassis chassis, RobotFeedback feedback) {
        return new ParallelGroup(
                autoRevShooter(shooter, chassis, feedback),
                SpindexerCommands.smartFeedWithSpindexerContinuous(shooter, spindexer, intake, feedback)
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

    /**
     * TeleOp shooting at fixed RPM with feedback
     */
    public static Command teleopShootFixedRPM(Shooter shooter, Spindexer spindexer, Intake intake,
                                               double rpm, RobotFeedback feedback) {
        return new ParallelGroup(
                runShooterPID(shooter, rpm),
                SpindexerCommands.smartFeedWithSpindexerContinuous(shooter, spindexer, intake, feedback)
        );
    }

    // ========================================================================
    // COLOR-SORTED SHOOTING SEQUENCES (uses AprilTag IDs 21, 22, 23)
    // (Includes 60-degree mechanical offset for shooter spin-up clearance)
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
     * Color-sorted shooting at fixed RPM with feedback
     */
    public static Command shootAllBallsColorSorted(Shooter shooter, Spindexer spindexer,
                                                    Intake intake, SuperChassis chassis,
                                                    double rpm, RobotFeedback feedback) {
        return new ParallelGroup(
                runShooterPID(shooter, rpm),
                SpindexerCommands.smartFeedColorSorted(shooter, spindexer, intake, chassis, feedback)
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
     * Color-sorted shooting with auto-aim and feedback
     */
    public static Command shootAllBallsColorSortedAutoAim(Shooter shooter, Spindexer spindexer,
                                                           Intake intake, SuperChassis chassis,
                                                           RobotFeedback feedback) {
        return new ParallelGroup(
                autoRevShooter(shooter, chassis, feedback),
                SpindexerCommands.smartFeedColorSorted(shooter, spindexer, intake, chassis, feedback)
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
     * TeleOp color-sorted shooting at fixed RPM with feedback
     */
    public static Command teleopShootColorSorted(Shooter shooter, Spindexer spindexer,
                                                  Intake intake, SuperChassis chassis,
                                                  double rpm, RobotFeedback feedback) {
        return new ParallelGroup(
                runShooterPID(shooter, rpm),
                SpindexerCommands.smartFeedColorSortedContinuous(shooter, spindexer, intake, chassis, feedback)
        );
    }


    /**
     * TeleOp color-sorted shooting with auto-aim and feedback
     * Best option for competition with full feedback (LED + rumble).
     */
    public static Command teleopShootColorSortedAutoAim(Shooter shooter, Spindexer spindexer,
                                                         Intake intake, SuperChassis chassis,
                                                         RobotFeedback feedback) {
        return new ParallelGroup(
                autoRevShooter(shooter, chassis, feedback),
                SpindexerCommands.smartFeedColorSortedContinuous(shooter, spindexer, intake, chassis, feedback)
        );
    }

    // ========================================================================
    // CUSTOM SEQUENCE SHOOTING (for 2-driver mode)
    // ========================================================================

    /**
     * Shoot balls in a custom sequence defined by the operator.
     * Used in 2-driver mode where spindexer operator programs the shooting order.
     */
    public static Command teleopShootCustomSequence(Shooter shooter, Spindexer spindexer,
                                                     Intake intake, double rpm,
                                                     java.util.List<Integer> sequence,
                                                     RobotFeedback feedback) {
        return new ParallelGroup(
                runShooterPID(shooter, rpm, feedback),
                SpindexerCommands.smartFeedCustomSequence(shooter, spindexer, intake, sequence, feedback)
        );
    }

    // ========================================================================
    // HOOD-BASED AUTO-AIM (Fixed RPM, variable hood angle)
    // Uses hood angle for distance instead of RPM adjustment
    // ========================================================================

    /**
     * Auto-aim using hood angle adjustment.
     * Keeps shooter at fixed RPM and adjusts hood based on distance.
     * This is the preferred method for accurate distance shooting.
     */
    public static Command autoAimWithHood(Shooter shooter, Hood hood, SuperChassis chassis) {
        return autoAimWithHood(shooter, hood, chassis, null);
    }

    /**
     * Auto-aim using hood with feedback
     */
    public static Command autoAimWithHood(Shooter shooter, Hood hood, SuperChassis chassis,
                                           RobotFeedback feedback) {
        final boolean[] hasNotifiedReady = {false};
        double targetTPS = ShooterConstants.rpmToTicksPerSecond(HoodConstants.FIXED_SHOOTING_RPM);

        return new LambdaCommand()
                .named("autoAimWithHood")
                .requires(shooter)
                .requires(hood)
                .setStart(() -> {
                    hasNotifiedReady[0] = false;
                    shooter.toVelocity(targetTPS);
                })
                .setUpdate(() -> {
                    // Keep shooter at fixed RPM
                    shooter.toVelocity(targetTPS);

                    // Adjust hood based on distance
                    double distance = chassis.getDistanceToTag();
                    if (distance > 0) {
                        hood.setHoodForDistance(distance);
                    }

                    // Trigger feedback once when RPM is reached
                    if (shooter.atSetpoint() && !hasNotifiedReady[0]) {
                        if (feedback != null) {
                            feedback.onShooterAtRPM();
                        }
                        hasNotifiedReady[0] = true;
                    }

                    dev.nextftc.ftc.ActiveOpMode.telemetry().addData("Hood Dist", "%.1f in", distance);
                    dev.nextftc.ftc.ActiveOpMode.telemetry().addData("Hood Angle", "%.1f deg", hood.getAngleDegrees());
                    dev.nextftc.ftc.ActiveOpMode.telemetry().addData("Fixed RPM", "%.0f", HoodConstants.FIXED_SHOOTING_RPM);
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
     * TeleOp color-sorted shooting with hood-based auto-aim.
     * Uses fixed RPM and hood angle for distance adjustment.
     * Best option for competition with full feedback (LED + rumble).
     */
    public static Command teleopShootColorSortedWithHood(Shooter shooter, Hood hood, Spindexer spindexer,
                                                          Intake intake, SuperChassis chassis,
                                                          RobotFeedback feedback) {
        return new ParallelGroup(
                autoAimWithHood(shooter, hood, chassis, feedback),
                SpindexerCommands.smartFeedColorSortedContinuous(shooter, spindexer, intake, chassis, feedback)
        );
    }

    /**
     * Full shooting routine with hood-based auto-aim.
     * Shoots until all balls are fired.
     */
    public static Command shootAllBallsWithHood(Shooter shooter, Hood hood, Spindexer spindexer,
                                                 Intake intake, SuperChassis chassis,
                                                 RobotFeedback feedback) {
        return new ParallelGroup(
                autoAimWithHood(shooter, hood, chassis, feedback),
                SpindexerCommands.smartFeedColorSorted(shooter, spindexer, intake, chassis, feedback)
        );
    }
}
