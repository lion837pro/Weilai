package org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.SuperChassis;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.ShooterConstants;

import java.util.function.DoubleSupplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.Delay;
import dev.nextftc.core.commands.utility.LambdaCommand;

/**
 * Commands for the Spindexer subsystem.
 *
 * Provides integration with Intake and Shooter subsystems for:
 * - Automatic ball loading during intake
 * - Smart feeding to shooter based on shooter readiness
 * - Sequential multi-ball shooting
 * - Manual control for testing
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
                    // Check if limit switch triggered
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
                .setUpdate(() -> {})  // periodic() handles the control loop
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

    // ========================================================================
    // INTAKE INTEGRATION COMMANDS
    // ========================================================================

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
     * Smart intake command - runs intake and auto-indexes spindexer when ball detected.
     * Stops when spindexer is full.
     */
    public static Command smartIntake(Spindexer spindexer, Intake intake, double intakeSpeed) {
        return new LambdaCommand()
                .named("smartIntake")
                .requires(spindexer)
                .requires(intake)
                .setStart(() -> {
                    // Move to intake position
                    spindexer.goToNextIntakePosition();
                })
                .setUpdate(() -> {
                    // Don't run intake if full
                    if (spindexer.isFull()) {
                        intake.MoveIn(0);
                        return;
                    }

                    // Ensure at intake position
                    if (!spindexer.atPosition()) {
                        intake.MoveIn(0);  // Wait for spindexer to settle
                        return;
                    }

                    // Run intake
                    intake.MoveIn(intakeSpeed);

                    // Check if ball was loaded (detected by sensor)
                    // The spindexer's periodic() handles ball detection
                    // If current slot is now loaded, move to next empty slot
                    int currentSlot = spindexer.getCurrentPosition() / 2;
                    if (spindexer.hasBall(currentSlot)) {
                        spindexer.goToNextIntakePosition();
                    }
                })
                .setStop(interrupted -> {
                    intake.MoveIn(0);
                    spindexer.stop();
                })
                .setIsDone(() -> spindexer.isFull())
                .setInterruptible(true);
    }

    /**
     * Continuous intake with spindexer - keeps running until interrupted.
     * Automatically indexes when balls are detected.
     */
    public static Command runIntakeWithSpindexer(Spindexer spindexer, Intake intake, double intakeSpeed) {
        return new LambdaCommand()
                .named("runIntakeWithSpindexer")
                .requires(spindexer)
                .requires(intake)
                .setStart(() -> spindexer.goToNextIntakePosition())
                .setUpdate(() -> {
                    if (spindexer.isFull()) {
                        intake.MoveIn(0);
                        return;
                    }

                    if (spindexer.atPosition() && spindexer.isAtIntakePosition()) {
                        intake.MoveIn(intakeSpeed);

                        // Auto-index when ball detected
                        int currentSlot = spindexer.getCurrentPosition() / 2;
                        if (spindexer.hasBall(currentSlot)) {
                            spindexer.goToNextIntakePosition();
                        }
                    } else {
                        intake.MoveIn(0);
                    }
                })
                .setStop(interrupted -> {
                    intake.MoveIn(0);
                    spindexer.stop();
                })
                .setIsDone(() -> false)  // Runs until interrupted
                .setInterruptible(true);
    }

    // ========================================================================
    // SHOOTER INTEGRATION COMMANDS
    // ========================================================================

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

    /**
     * Smart feed to shooter - feeds ball only when shooter is at setpoint,
     * then indexes to next loaded slot.
     */
    public static Command smartFeedWithSpindexer(Shooter shooter, Spindexer spindexer, Intake intake) {
        return new LambdaCommand()
                .named("smartFeedWithSpindexer")
                .requires(spindexer)
                .requires(intake)
                .setStart(() -> spindexer.goToNextShooterPosition())
                .setUpdate(() -> {
                    // No balls to shoot
                    if (spindexer.isEmpty()) {
                        intake.MoveIn(0);
                        return;
                    }

                    // Wait for spindexer to be at shooter position
                    if (!spindexer.atPosition() || !spindexer.isAtShooterPosition()) {
                        intake.MoveIn(0);
                        return;
                    }

                    // Wait for shooter to be ready
                    if (shooter.atSetpoint()) {
                        // Feed the ball!
                        intake.MoveIn(1.0);

                        // Mark slot as empty and index to next
                        spindexer.markCurrentSlotEmpty();
                        spindexer.goToNextShooterPosition();
                    } else {
                        intake.MoveIn(0);  // Wait for spin-up
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
     * Shoot single ball - indexes to shooter position, waits for shooter ready, feeds once.
     */
    public static Command shootSingleBall(Shooter shooter, Spindexer spindexer, Intake intake) {
        final boolean[] hasFed = {false};

        return new LambdaCommand()
                .named("shootSingleBall")
                .requires(spindexer)
                .requires(intake)
                .setStart(() -> {
                    hasFed[0] = false;
                    spindexer.goToNextShooterPosition();
                })
                .setUpdate(() -> {
                    if (hasFed[0]) return;  // Already fired

                    if (spindexer.isEmpty()) {
                        hasFed[0] = true;
                        return;
                    }

                    if (!spindexer.atPosition() || !spindexer.isAtShooterPosition()) {
                        intake.MoveIn(0);
                        return;
                    }

                    if (shooter.atSetpoint()) {
                        intake.MoveIn(1.0);
                        spindexer.markCurrentSlotEmpty();
                        hasFed[0] = true;
                    } else {
                        intake.MoveIn(0);
                    }
                })
                .setStop(interrupted -> {
                    intake.MoveIn(0);
                })
                .setIsDone(() -> hasFed[0])
                .setInterruptible(true);
    }

    // ========================================================================
    // COMPOSITE COMMANDS - FULL SHOOTING SEQUENCES
    // ========================================================================

    /**
     * Full shooting routine with spindexer at fixed RPM.
     * Revs shooter to target RPM and smart-feeds all balls.
     */
    public static Command shootAllBalls(Shooter shooter, Spindexer spindexer, Intake intake, double rpm) {
        return new ParallelGroup(
                // Run shooter at target RPM
                runShooterPID(shooter, rpm),
                // Smart feed with spindexer indexing
                smartFeedWithSpindexer(shooter, spindexer, intake)
        );
    }

    /**
     * Full shooting routine with auto-aim.
     * Uses vision to calculate optimal RPM.
     */
    public static Command shootAllBallsWithAutoAim(Shooter shooter, Spindexer spindexer,
                                                    Intake intake, SuperChassis chassis) {
        return new ParallelGroup(
                // Auto-rev based on distance
                autoRevShooter(shooter, chassis),
                // Smart feed with spindexer
                smartFeedWithSpindexer(shooter, spindexer, intake)
        );
    }

    /**
     * Rapid fire all balls - continuous feeding at fixed RPM.
     * Less accurate but faster than smart feed.
     */
    public static Command rapidFireAll(Shooter shooter, Spindexer spindexer, Intake intake, double rpm) {
        return new ParallelGroup(
                runShooterPID(shooter, rpm),
                rapidFeedAll(spindexer, intake)
        );
    }

    /**
     * Rapid feed helper - continuously feeds without waiting for shooter setpoint.
     */
    private static Command rapidFeedAll(Spindexer spindexer, Intake intake) {
        return new LambdaCommand()
                .named("rapidFeedAll")
                .requires(spindexer)
                .requires(intake)
                .setStart(() -> spindexer.goToNextShooterPosition())
                .setUpdate(() -> {
                    if (spindexer.isEmpty()) {
                        intake.MoveIn(0);
                        return;
                    }

                    if (spindexer.atPosition() && spindexer.isAtShooterPosition()) {
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

    // ========================================================================
    // SHOOTER HELPER COMMANDS (copied from ShooterCommands for convenience)
    // ========================================================================

    /**
     * Run shooter at specific RPM using PID control.
     */
    private static Command runShooterPID(Shooter shooter, double rpm) {
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
     * Auto-rev shooter based on vision distance.
     */
    private static Command autoRevShooter(Shooter shooter, SuperChassis chassis) {
        return new LambdaCommand()
                .named("autoRevShooter")
                .requires(shooter)
                .setUpdate(() -> {
                    double distance = chassis.getDistanceToTag();
                    double targetRPM = ShooterConstants.distanceToRPM(distance);

                    // Safety clamps
                    if (targetRPM > ShooterConstants.MAX_SHOOTING_RPM) {
                        targetRPM = ShooterConstants.MAX_SHOOTING_RPM;
                    }
                    if (distance <= 0) {
                        targetRPM = ShooterConstants.MIN_SHOOTING_RPM;
                    }

                    double targetTPS = ShooterConstants.rpmToTicksPerSecond(targetRPM);
                    shooter.toVelocity(targetTPS);
                })
                .setStop(interrupted -> shooter.stop())
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    // ========================================================================
    // TELEOP COMPOSITE COMMANDS
    // ========================================================================

    /**
     * Main TeleOp intake command - runs intake while auto-indexing spindexer.
     * Button press starts, release stops.
     */
    public static Command teleopIntake(Spindexer spindexer, Intake intake, double speed) {
        return new LambdaCommand()
                .named("teleopIntake")
                .requires(spindexer)
                .requires(intake)
                .setStart(() -> {
                    if (!spindexer.isFull()) {
                        spindexer.goToNextIntakePosition();
                    }
                })
                .setUpdate(() -> {
                    if (spindexer.isFull()) {
                        intake.MoveIn(0);
                        return;
                    }

                    if (spindexer.atPosition() && spindexer.isAtIntakePosition()) {
                        intake.MoveIn(speed);

                        int currentSlot = spindexer.getCurrentPosition() / 2;
                        if (spindexer.hasBall(currentSlot)) {
                            spindexer.goToNextIntakePosition();
                        }
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

    /**
     * Main TeleOp shoot command - shoots all loaded balls with auto-aim.
     */
    public static Command teleopShoot(Shooter shooter, Spindexer spindexer,
                                       Intake intake, SuperChassis chassis) {
        return shootAllBallsWithAutoAim(shooter, spindexer, intake, chassis);
    }

    /**
     * Main TeleOp shoot command at fixed RPM.
     */
    public static Command teleopShootFixedRPM(Shooter shooter, Spindexer spindexer,
                                               Intake intake, double rpm) {
        return shootAllBalls(shooter, spindexer, intake, rpm);
    }

    /**
     * Stop all - stops spindexer, intake, and shooter.
     */
    public static Command stopAll(Shooter shooter, Spindexer spindexer, Intake intake) {
        return new LambdaCommand()
                .named("stopAll")
                .requires(shooter)
                .requires(spindexer)
                .requires(intake)
                .setStart(() -> {
                    shooter.stop();
                    spindexer.stop();
                    intake.MoveIn(0);
                })
                .setIsDone(() -> true)
                .setInterruptible(true);
    }
}
