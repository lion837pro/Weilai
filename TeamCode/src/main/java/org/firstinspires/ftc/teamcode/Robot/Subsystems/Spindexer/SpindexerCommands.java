package org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.SuperChassis;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.VisionConstants;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.VisionConstants.BallColor;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.LED.RobotFeedback;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.Shooter;

import java.util.function.DoubleSupplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.ftc.ActiveOpMode;

/**
 * Commands for the Spindexer subsystem.
 * Contains spindexer-specific commands and smart feed helpers.
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
    // With 60-degree offset for mechanical clearance during shooter spin-up
    // ========================================================================

    /**
     * Shooting state machine states
     */
    private enum ShootState {
        MOVING_TO_SHOOTER,      // Moving to shooter position
        APPLYING_OFFSET,        // Applying 60-degree offset for shooter clearance
        WAITING_FOR_SPINUP,     // Waiting for shooter to reach RPM
        REMOVING_OFFSET,        // Moving ball back to shooter position
        FEEDING,                // Ball is being fed to shooter
        NEXT_BALL              // Moving to next ball
    }

    /**
     * Smart feed with spindexer indexing and mechanical offset.
     * Sequence: Move to shooter pos → Offset 60° → Wait for RPM → Return → Feed
     * Runs until all balls are fired.
     */
    public static Command smartFeedWithSpindexer(Shooter shooter, Spindexer spindexer, Intake intake) {
        return smartFeedWithSpindexerInternal(shooter, spindexer, intake, null, false);
    }

    /**
     * Smart feed with feedback (LED + rumble)
     */
    public static Command smartFeedWithSpindexer(Shooter shooter, Spindexer spindexer,
                                                  Intake intake, RobotFeedback feedback) {
        return smartFeedWithSpindexerInternal(shooter, spindexer, intake, feedback, false);
    }

    /**
     * Continuous smart feed for TeleOp - runs until interrupted.
     */
    public static Command smartFeedWithSpindexerContinuous(Shooter shooter, Spindexer spindexer, Intake intake) {
        return smartFeedWithSpindexerInternal(shooter, spindexer, intake, null, true);
    }

    /**
     * Continuous smart feed with feedback
     */
    public static Command smartFeedWithSpindexerContinuous(Shooter shooter, Spindexer spindexer,
                                                            Intake intake, RobotFeedback feedback) {
        return smartFeedWithSpindexerInternal(shooter, spindexer, intake, feedback, true);
    }

    /**
     * Internal smart feed implementation with state machine
     */
    private static Command smartFeedWithSpindexerInternal(Shooter shooter, Spindexer spindexer,
                                                           Intake intake, RobotFeedback feedback,
                                                           boolean continuous) {
        final ShootState[] state = {ShootState.MOVING_TO_SHOOTER};
        final BallColor[] currentBallColor = {BallColor.UNKNOWN};

        return new LambdaCommand()
                .named(continuous ? "smartFeedContinuous" : "smartFeed")
                .requires(spindexer)
                .requires(intake)
                .setStart(() -> {
                    state[0] = ShootState.MOVING_TO_SHOOTER;
                    spindexer.resetOffsetState();
                    spindexer.goToNextShooterPosition();

                    // Get color of next ball for LED feedback
                    int slot = getNextLoadedSlot(spindexer);
                    if (slot >= 0) {
                        currentBallColor[0] = spindexer.getBallColor(slot);
                        if (feedback != null) {
                            feedback.onShooterSpinUp(currentBallColor[0]);
                        }
                    }
                })
                .setUpdate(() -> {
                    if (spindexer.isEmpty()) {
                        intake.MoveIn(0);
                        if (feedback != null) feedback.onSpindexerEmpty();
                        return;
                    }

                    switch (state[0]) {
                        case MOVING_TO_SHOOTER:
                            intake.MoveIn(0);
                            if (spindexer.atPosition() && spindexer.isAtShooterPosition()) {
                                // At shooter position, apply offset
                                spindexer.applyShooterOffset();
                                state[0] = ShootState.APPLYING_OFFSET;
                            }
                            break;

                        case APPLYING_OFFSET:
                            intake.MoveIn(0);
                            if (spindexer.atPosition()) {
                                // Offset applied, now wait for shooter spin-up
                                state[0] = ShootState.WAITING_FOR_SPINUP;
                            }
                            break;

                        case WAITING_FOR_SPINUP:
                            intake.MoveIn(0);
                            if (shooter.atSetpoint()) {
                                // Shooter at RPM, remove offset to bring ball to shooter
                                spindexer.removeShooterOffset();
                                state[0] = ShootState.REMOVING_OFFSET;
                            }
                            break;

                        case REMOVING_OFFSET:
                            intake.MoveIn(0);
                            if (spindexer.atPosition()) {
                                // Ball is at shooter position, start feeding
                                state[0] = ShootState.FEEDING;
                            }
                            break;

                        case FEEDING:
                            intake.MoveIn(1.0);  // Feed the ball

                            // Mark ball as shot and provide feedback
                            if (feedback != null) {
                                feedback.onBallShot(currentBallColor[0]);
                            }

                            spindexer.markCurrentSlotEmpty();
                            state[0] = ShootState.NEXT_BALL;
                            break;

                        case NEXT_BALL:
                            intake.MoveIn(0);
                            if (!spindexer.isEmpty()) {
                                // Move to next ball
                                spindexer.goToNextShooterPosition();
                                state[0] = ShootState.MOVING_TO_SHOOTER;

                                // Update ball color for feedback
                                int slot = getNextLoadedSlot(spindexer);
                                if (slot >= 0) {
                                    currentBallColor[0] = spindexer.getBallColor(slot);
                                    if (feedback != null) {
                                        feedback.onShooterSpinUp(currentBallColor[0]);
                                    }
                                }
                            }
                            break;
                    }
                })
                .setStop(interrupted -> {
                    intake.MoveIn(0);
                    spindexer.stop();
                    spindexer.resetOffsetState();
                    if (feedback != null) feedback.onShooterStop();
                })
                .setIsDone(() -> !continuous && spindexer.isEmpty())
                .setInterruptible(true);
    }

    /**
     * Get the next loaded slot index
     */
    private static int getNextLoadedSlot(Spindexer spindexer) {
        for (int i = 0; i < SpindexerConstants.SLOTS_COUNT; i++) {
            if (spindexer.hasBall(i)) {
                return i;
            }
        }
        return -1;
    }

    // ========================================================================
    // COLOR-SORTED SMART FEED (uses AprilTag IDs 21, 22, 23)
    // With 60-degree offset for mechanical clearance
    // ========================================================================

    /**
     * Smart feed with color sorting based on AprilTag.
     * Uses detected tag ID (21, 22, 23) to determine shooting order:
     * - Tag 21 (G,P,P): Shoot GREEN first from slot 0
     * - Tag 22 (P,G,P): Shoot GREEN first from slot 1
     * - Tag 23 (P,P,G): Shoot GREEN first from slot 2

     * If no color sort tag is detected, falls back to normal sequential shooting.
     * Includes 60-degree offset for mechanical clearance during shooter spin-up.
     */
    public static Command smartFeedColorSorted(Shooter shooter, Spindexer spindexer,
                                                Intake intake, SuperChassis chassis) {
        return smartFeedColorSortedInternal(shooter, spindexer, intake, chassis, null, false);
    }

    /**
     * Color-sorted smart feed with feedback
     */
    public static Command smartFeedColorSorted(Shooter shooter, Spindexer spindexer,
                                                Intake intake, SuperChassis chassis,
                                                RobotFeedback feedback) {
        return smartFeedColorSortedInternal(shooter, spindexer, intake, chassis, feedback, false);
    }

    /**
     * Continuous color-sorted smart feed for TeleOp.
     */
    public static Command smartFeedColorSortedContinuous(Shooter shooter, Spindexer spindexer,
                                                          Intake intake, SuperChassis chassis) {
        return smartFeedColorSortedInternal(shooter, spindexer, intake, chassis, null, true);
    }

    /**
     * Continuous color-sorted smart feed with feedback
     */
    public static Command smartFeedColorSortedContinuous(Shooter shooter, Spindexer spindexer,
                                                          Intake intake, SuperChassis chassis,
                                                          RobotFeedback feedback) {
        return smartFeedColorSortedInternal(shooter, spindexer, intake, chassis, feedback, true);
    }

    /**
     * Internal implementation of color-sorted smart feed with state machine
     */
    private static Command smartFeedColorSortedInternal(Shooter shooter, Spindexer spindexer,
                                                         Intake intake, SuperChassis chassis,
                                                         RobotFeedback feedback, boolean continuous) {
        final int[] targetSlotOrder = {-1, -1, -1};
        final int[] currentIndex = {0};
        final ShootState[] state = {ShootState.MOVING_TO_SHOOTER};
        final BallColor[] currentBallColor = {BallColor.UNKNOWN};
        final int[] lastTagId = {-1};

        return new LambdaCommand()
                .named(continuous ? "colorSortedFeedContinuous" : "colorSortedFeed")
                .requires(spindexer)
                .requires(intake)
                .setStart(() -> {
                    currentIndex[0] = 0;
                    state[0] = ShootState.MOVING_TO_SHOOTER;
                    spindexer.resetOffsetState();

                    // Determine shooting order based on locked or detected tag
                    int tagId = chassis.getColorSortTag();
                    lastTagId[0] = tagId;
                    int greenSlot = VisionConstants.getGreenSlotForTag(tagId);

                    if (greenSlot != -1) {
                        targetSlotOrder[0] = greenSlot;
                        int idx = 1;
                        for (int i = 0; i < 3; i++) {
                            if (i != greenSlot) {
                                targetSlotOrder[idx++] = i;
                            }
                        }
                        ActiveOpMode.telemetry().addData("ColorSort", "Tag %d: Green at slot %d", tagId, greenSlot);
                    } else {
                        targetSlotOrder[0] = 0;
                        targetSlotOrder[1] = 1;
                        targetSlotOrder[2] = 2;
                    }

                    // Move to first slot with a ball
                    goToNextSlotInOrder(spindexer, targetSlotOrder, currentIndex);

                    // Get color for feedback
                    if (currentIndex[0] < 3) {
                        int slot = targetSlotOrder[currentIndex[0]];
                        currentBallColor[0] = spindexer.getBallColor(slot);
                        if (feedback != null) {
                            feedback.onShooterSpinUp(currentBallColor[0]);
                        }
                    }
                })
                .setUpdate(() -> {
                    if (spindexer.isEmpty()) {
                        intake.MoveIn(0);
                        if (feedback != null) feedback.onSpindexerEmpty();
                        return;
                    }

                    // Note: Color sort order is now persistent via chassis.getColorSortTag()
                    // Use the override button to change the locked tag

                    switch (state[0]) {
                        case MOVING_TO_SHOOTER:
                            intake.MoveIn(0);
                            if (spindexer.atPosition() && spindexer.isAtShooterPosition()) {
                                spindexer.applyShooterOffset();
                                state[0] = ShootState.APPLYING_OFFSET;
                            }
                            break;

                        case APPLYING_OFFSET:
                            intake.MoveIn(0);
                            if (spindexer.atPosition()) {
                                state[0] = ShootState.WAITING_FOR_SPINUP;
                            }
                            break;

                        case WAITING_FOR_SPINUP:
                            intake.MoveIn(0);
                            if (shooter.atSetpoint()) {
                                spindexer.removeShooterOffset();
                                state[0] = ShootState.REMOVING_OFFSET;
                            }
                            break;

                        case REMOVING_OFFSET:
                            intake.MoveIn(0);
                            if (spindexer.atPosition()) {
                                state[0] = ShootState.FEEDING;
                            }
                            break;

                        case FEEDING:
                            intake.MoveIn(1.0);

                            if (feedback != null) {
                                feedback.onBallShot(currentBallColor[0]);
                            }

                            spindexer.markCurrentSlotEmpty();
                            currentIndex[0]++;
                            state[0] = ShootState.NEXT_BALL;
                            break;

                        case NEXT_BALL:
                            intake.MoveIn(0);
                            if (!spindexer.isEmpty()) {
                                goToNextSlotInOrder(spindexer, targetSlotOrder, currentIndex);
                                state[0] = ShootState.MOVING_TO_SHOOTER;

                                // Update ball color for feedback
                                if (currentIndex[0] < 3) {
                                    int slot = targetSlotOrder[currentIndex[0]];
                                    if (spindexer.hasBall(slot)) {
                                        currentBallColor[0] = spindexer.getBallColor(slot);
                                        if (feedback != null) {
                                            feedback.onShooterSpinUp(currentBallColor[0]);
                                        }
                                    }
                                }
                            }
                            break;
                    }

                    // Telemetry
                    BallColor[] colors = spindexer.getAllBallColors();
                    ActiveOpMode.telemetry().addData("Balls", "[%s %s %s]",
                            colorChar(colors[0]), colorChar(colors[1]), colorChar(colors[2]));
                    ActiveOpMode.telemetry().addData("ShootState", state[0].toString());
                    ActiveOpMode.telemetry().addData("Color Sort Mode", chassis.getColorSortModeString());
                })
                .setStop(interrupted -> {
                    intake.MoveIn(0);
                    spindexer.stop();
                    spindexer.resetOffsetState();
                    if (feedback != null) feedback.onShooterStop();
                })
                .setIsDone(() -> !continuous && spindexer.isEmpty())
                .setInterruptible(true);
    }

    // ========================================================================
    // HELPER METHODS
    // ========================================================================

    /**
     * Move to the next slot in the specified order that has a ball.
     */
    private static void goToNextSlotInOrder(Spindexer spindexer, int[] slotOrder, int[] currentIndex) {
        while (currentIndex[0] < 3) {
            int slot = slotOrder[currentIndex[0]];
            if (spindexer.hasBall(slot)) {
                int shooterPos = SpindexerConstants.getShooterPosition(slot);
                spindexer.goToPosition(shooterPos);
                return;
            }
            currentIndex[0]++;
        }
        // No more balls
    }

    private static String colorChar(BallColor color) {
        switch (color) {
            case GREEN: return "G";
            case PURPLE: return "P";
            default: return "?";
        }
    }

    // ========================================================================
    // CUSTOM SEQUENCE SMART FEED (for 2-driver mode)
    // ========================================================================

    /**
     * Smart feed with custom sequence programmed by operator.
     * Shoots balls in the order specified in the sequence list.
     * Used in 2-driver mode where spindexer operator programs shooting order.
     *
     * @param shooter The shooter subsystem
     * @param spindexer The spindexer subsystem
     * @param intake The intake subsystem
     * @param sequence List of slot indices (0-2) in desired shooting order
     * @param feedback Robot feedback for LED and rumble
     */
    public static Command smartFeedCustomSequence(Shooter shooter, Spindexer spindexer,
                                                   Intake intake, java.util.List<Integer> sequence,
                                                   RobotFeedback feedback) {
        final int[] currentIndex = {0};
        final ShootState[] state = {ShootState.MOVING_TO_SHOOTER};
        final BallColor[] currentBallColor = {BallColor.UNKNOWN};

        return new LambdaCommand()
                .named("customSequenceFeed")
                .requires(spindexer)
                .requires(intake)
                .setStart(() -> {
                    currentIndex[0] = 0;
                    state[0] = ShootState.MOVING_TO_SHOOTER;
                    spindexer.resetOffsetState();

                    // If sequence is empty or spindexer is empty, do nothing
                    if (sequence.isEmpty() || spindexer.isEmpty()) {
                        return;
                    }

                    // Move to first slot in sequence
                    int firstSlot = sequence.get(0);
                    if (firstSlot >= 0 && firstSlot < 3) {
                        int shooterPos = SpindexerConstants.getShooterPosition(firstSlot);
                        spindexer.goToPosition(shooterPos);

                        // Get color for feedback
                        currentBallColor[0] = spindexer.getBallColor(firstSlot);
                        if (feedback != null) {
                            feedback.onShooterSpinUp(currentBallColor[0]);
                        }
                    }
                })
                .setUpdate(() -> {
                    // Check if sequence is empty or we're done
                    if (sequence.isEmpty() || currentIndex[0] >= sequence.size()) {
                        intake.MoveIn(0);
                        if (feedback != null) feedback.onSpindexerEmpty();
                        return;
                    }

                    switch (state[0]) {
                        case MOVING_TO_SHOOTER:
                            intake.MoveIn(0);
                            if (spindexer.atPosition() && spindexer.isAtShooterPosition()) {
                                spindexer.applyShooterOffset();
                                state[0] = ShootState.APPLYING_OFFSET;
                            }
                            break;

                        case APPLYING_OFFSET:
                            intake.MoveIn(0);
                            if (spindexer.atPosition()) {
                                state[0] = ShootState.WAITING_FOR_SPINUP;
                            }
                            break;

                        case WAITING_FOR_SPINUP:
                            intake.MoveIn(0);
                            if (shooter.atSetpoint()) {
                                spindexer.removeShooterOffset();
                                state[0] = ShootState.REMOVING_OFFSET;
                            }
                            break;

                        case REMOVING_OFFSET:
                            intake.MoveIn(0);
                            if (spindexer.atPosition()) {
                                state[0] = ShootState.FEEDING;
                            }
                            break;

                        case FEEDING:
                            intake.MoveIn(1.0);

                            if (feedback != null) {
                                feedback.onBallShot(currentBallColor[0]);
                            }

                            spindexer.markCurrentSlotEmpty();
                            currentIndex[0]++;
                            state[0] = ShootState.NEXT_BALL;
                            break;

                        case NEXT_BALL:
                            intake.MoveIn(0);
                            // Check if there are more balls in sequence
                            if (currentIndex[0] < sequence.size()) {
                                int nextSlot = sequence.get(currentIndex[0]);
                                if (nextSlot >= 0 && nextSlot < 3 && spindexer.hasBall(nextSlot)) {
                                    // Move to next slot
                                    int shooterPos = SpindexerConstants.getShooterPosition(nextSlot);
                                    spindexer.goToPosition(shooterPos);
                                    state[0] = ShootState.MOVING_TO_SHOOTER;

                                    // Update ball color for feedback
                                    currentBallColor[0] = spindexer.getBallColor(nextSlot);
                                    if (feedback != null) {
                                        feedback.onShooterSpinUp(currentBallColor[0]);
                                    }
                                } else {
                                    // Slot empty or invalid, skip to next
                                    currentIndex[0]++;
                                }
                            }
                            break;
                    }

                    // Telemetry
                    BallColor[] colors = spindexer.getAllBallColors();
                    ActiveOpMode.telemetry().addData("Balls", "[%s %s %s]",
                            colorChar(colors[0]), colorChar(colors[1]), colorChar(colors[2]));
                    ActiveOpMode.telemetry().addData("ShootState", state[0].toString());
                    ActiveOpMode.telemetry().addData("Custom Sequence", formatSequence(sequence));
                    ActiveOpMode.telemetry().addData("Current Shot", "%d / %d",
                            currentIndex[0] + 1, sequence.size());
                })
                .setStop(interrupted -> {
                    intake.MoveIn(0);
                    spindexer.stop();
                    spindexer.resetOffsetState();
                    if (feedback != null) feedback.onShooterStop();
                })
                .setIsDone(() -> currentIndex[0] >= sequence.size())
                .setInterruptible(true);
    }

    /**
     * Format sequence for telemetry display
     */
    private static String formatSequence(java.util.List<Integer> sequence) {
        if (sequence.isEmpty()) {
            return "[Empty]";
        }
        StringBuilder sb = new StringBuilder("[");
        for (int i = 0; i < sequence.size(); i++) {
            sb.append(sequence.get(i));
            if (i < sequence.size() - 1) {
                sb.append(" → ");
            }
        }
        sb.append("]");
        return sb.toString();
    }
}
