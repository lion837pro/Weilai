package org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.Shooter;

import java.util.function.DoubleSupplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;

/**
 * Commands for the Spindexer subsystem.
 * Contains only spindexer-specific commands.
 *
 * For intake sequences, see IntakeCommands.
 * For shooting sequences, see ShooterCommands.
 */
public class SpindexerCommands {

    // ========================================================================
    // BASIC SPINDEXER COMMANDS
    // ========================================================================

    /**
     * Home the spindexer using the magnetic limit switch.
     * Must be run before using position-based commands.
     */
    public static Command homeSpindexer(Spindexer spindexer) {
        return new LambdaCommand()
                .named("homeSpindexer")
                .requires(spindexer)
                .setStart(() -> spindexer.startHoming())
                .setUpdate(() -> {
                    if (spindexer.isAtHome()) {
                        spindexer.finishHoming();
                    }
                })
                .setStop(interrupted -> {
                    if (interrupted) {
                        spindexer.stop();
                    }
                })
                .setIsDone(() -> spindexer.isHomed())
                .setInterruptible(true);
    }

    /**
     * Move spindexer to a specific position (0-5)
     */
    public static Command goToPosition(Spindexer spindexer, int position) {
        return new LambdaCommand()
                .named("goToPosition_" + position)
                .requires(spindexer)
                .setStart(() -> spindexer.goToPosition(position))
                .setUpdate(() -> {})
                .setStop(interrupted -> {
                    if (interrupted) spindexer.stop();
                })
                .setIsDone(() -> spindexer.atPosition())
                .setInterruptible(true);
    }

    /**
     * Index forward by one position
     */
    public static Command indexForward(Spindexer spindexer) {
        return new LambdaCommand()
                .named("indexForward")
                .requires(spindexer)
                .setStart(() -> spindexer.indexForward())
                .setUpdate(() -> {})
                .setStop(interrupted -> {
                    if (interrupted) spindexer.stop();
                })
                .setIsDone(() -> spindexer.atPosition())
                .setInterruptible(true);
    }

    /**
     * Index backward by one position
     */
    public static Command indexBackward(Spindexer spindexer) {
        return new LambdaCommand()
                .named("indexBackward")
                .requires(spindexer)
                .setStart(() -> spindexer.indexBackward())
                .setUpdate(() -> {})
                .setStop(interrupted -> {
                    if (interrupted) spindexer.stop();
                })
                .setIsDone(() -> spindexer.atPosition())
                .setInterruptible(true);
    }

    /**
     * Stop the spindexer
     */
    public static Command stopSpindexer(Spindexer spindexer) {
        return new LambdaCommand()
                .named("stopSpindexer")
                .requires(spindexer)
                .setStart(() -> spindexer.stop())
                .setIsDone(() -> true)
                .setInterruptible(true);
    }

    /**
     * Manual spin control (for testing or clearing jams)
     */
    public static Command manualSpin(Spindexer spindexer, DoubleSupplier powerSupplier) {
        return new LambdaCommand()
                .named("manualSpin")
                .requires(spindexer)
                .setUpdate(() -> {
                    double power = powerSupplier.getAsDouble();
                    if (Math.abs(power) < 0.1) power = 0;
                    spindexer.spin(power);
                })
                .setStop(interrupted -> spindexer.stop())
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    /**
     * Position spindexer for intake - moves to next empty slot at intake position
     */
    public static Command prepareForIntake(Spindexer spindexer) {
        return new LambdaCommand()
                .named("prepareForIntake")
                .requires(spindexer)
                .setStart(() -> spindexer.goToNextIntakePosition())
                .setUpdate(() -> {})
                .setStop(interrupted -> {
                    if (interrupted) spindexer.stop();
                })
                .setIsDone(() -> spindexer.atPosition())
                .setInterruptible(true);
    }

    /**
     * Position spindexer for shooting - moves to next loaded slot at shooter position
     */
    public static Command prepareForShoot(Spindexer spindexer) {
        return new LambdaCommand()
                .named("prepareForShoot")
                .requires(spindexer)
                .setStart(() -> spindexer.goToNextShooterPosition())
                .setUpdate(() -> {})
                .setStop(interrupted -> {
                    if (interrupted) spindexer.stop();
                })
                .setIsDone(() -> spindexer.atPosition())
                .setInterruptible(true);
    }

    // ========================================================================
    // SMART FEED HELPERS (used by ShooterCommands for shooting sequences)
    // ========================================================================

    /**
     * Smart feed with spindexer indexing - feeds ball only when shooter is at setpoint,
     * then indexes to next loaded slot. Runs until all balls are fired.
     */
    public static Command smartFeedWithSpindexer(Shooter shooter, Spindexer spindexer, Intake intake) {
        return new LambdaCommand()
                .named("smartFeedWithSpindexer")
                .requires(spindexer)
                .requires(intake)
                .setStart(() -> spindexer.goToNextShooterPosition())
                .setUpdate(() -> {
                    if (spindexer.isEmpty()) {
                        intake.MoveIn(0);
                        return;
                    }

                    if (!spindexer.atPosition() || !spindexer.isAtShooterPosition()) {
                        intake.MoveIn(0);
                        return;
                    }

                    if (shooter.atSetpoint()) {
                        intake.MoveIn(1.0);
                        spindexer.markCurrentSlotEmpty();
                        spindexer.goToNextShooterPosition();
                    } else {
                        intake.MoveIn(0);
                    }
                })
                .setStop(interrupted -> {
                    intake.MoveIn(0);
                    spindexer.stop();
                })
                .setIsDone(() -> spindexer.isEmpty())
                .setInterruptible(true);
    }

    /**
     * Continuous smart feed for TeleOp - runs until interrupted.
     */
    public static Command smartFeedWithSpindexerContinuous(Shooter shooter, Spindexer spindexer, Intake intake) {
        return new LambdaCommand()
                .named("smartFeedWithSpindexerContinuous")
                .requires(spindexer)
                .requires(intake)
                .setStart(() -> spindexer.goToNextShooterPosition())
                .setUpdate(() -> {
                    if (spindexer.isEmpty()) {
                        intake.MoveIn(0);
                        return;
                    }

                    if (!spindexer.atPosition() || !spindexer.isAtShooterPosition()) {
                        intake.MoveIn(0);
                        return;
                    }

                    if (shooter.atSetpoint()) {
                        intake.MoveIn(1.0);
                        spindexer.markCurrentSlotEmpty();
                        spindexer.goToNextShooterPosition();
                    } else {
                        intake.MoveIn(0);
                    }
                })
                .setStop(interrupted -> {
                    intake.MoveIn(0);
                    spindexer.stop();
                })
                .setIsDone(() -> false)
                .setInterruptible(true);
    }
}
