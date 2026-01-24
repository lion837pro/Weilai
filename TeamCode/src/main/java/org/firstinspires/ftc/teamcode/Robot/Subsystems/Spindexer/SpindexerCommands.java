package org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.SuperChassis;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.VisionConstants;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.VisionConstants.BallColor;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.LED.RobotFeedback;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.ShooterEmergency;

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
                .setStop(interrupted -> spindexer.stop())
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
     * Alias for prepareForIntake - moves to next empty slot at intake position
     */
    public static Command goToNextIntakePosition(Spindexer spindexer) {
        return prepareForIntake(spindexer);
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

    /**
     * Alias for prepareForShoot - moves to next loaded slot at shooter position
     */
    public static Command goToNextShooterPosition(Spindexer spindexer) {
        return prepareForShoot(spindexer);
    }

    // ========================================================================
    // FEEDER SERVO COMMANDS
    // Controls the servo that transfers ball from spindexer to shooter
    // ========================================================================

    /**
     * Move feeder servo UP (120 degrees) to push ball into shooter
     */
    public static Command feederUp(Spindexer spindexer) {
        final ElapsedTime timer = new ElapsedTime();

        return new LambdaCommand()
                .named("feederUp")
                .requires(spindexer)
                .setStart(() -> {
                    spindexer.feederUp();
                    timer.reset();
                })
                .setUpdate(() -> {})
                .setStop(interrupted -> {})
                .setIsDone(() -> timer.milliseconds() >= SpindexerConstants.FEEDER_MOVE_TIME_MS)
                .setInterruptible(true);
    }

    /**
     * Move feeder servo DOWN (0 degrees) - resting position
     */
    public static Command feederDown(Spindexer spindexer) {
        final ElapsedTime timer = new ElapsedTime();

        return new LambdaCommand()
                .named("feederDown")
                .requires(spindexer)
                .setStart(() -> {
                    spindexer.feederDown();
                    timer.reset();
                })
                .setUpdate(() -> {})
                .setStop(interrupted -> {})
                .setIsDone(() -> timer.milliseconds() >= SpindexerConstants.FEEDER_MOVE_TIME_MS)
                .setInterruptible(true);
    }

    /**
     * Feed sequence: Move feeder UP, wait, then DOWN
     * Complete cycle to push one ball into shooter
     */
    public static Command feedBall(Spindexer spindexer) {
        final ElapsedTime timer = new ElapsedTime();
        final boolean[] isUp = {false};

        return new LambdaCommand()
                .named("feedBall")
                .requires(spindexer)
                .setStart(() -> {
                    spindexer.feederUp();
                    timer.reset();
                    isUp[0] = true;
                })
                .setUpdate(() -> {
                    if (isUp[0] && timer.milliseconds() >= SpindexerConstants.FEEDER_MOVE_TIME_MS) {
                        // Feeder is up, now bring it down
                        spindexer.feederDown();
                        timer.reset();
                        isUp[0] = false;
                    }
                })
                .setStop(interrupted -> {
                    // Always return to down position when stopped
                    spindexer.feederDown();
                })
                .setIsDone(() -> !isUp[0] && timer.milliseconds() >= SpindexerConstants.FEEDER_MOVE_TIME_MS)
                .setInterruptible(true);
    }

    // ========================================================================
    // SMART FEED HELPERS (used by ShooterCommands for shooting sequences)
    // ========================================================================

    /**
     * Shooting state machine states
     */
    private enum ShootState {
        MOVING_TO_SHOOTER,      // Moving to shooter position
        WAITING_FOR_SPINUP,     // Waiting for shooter to reach RPM
        FEEDER_UP,              // Feeder servo pushing ball into shooter
        FEEDER_DOWN,            // Feeder servo returning to rest position
        NEXT_BALL               // Moving to next ball
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
        final ElapsedTime feedTimer = new ElapsedTime();
        final boolean[] feedTimerStarted = {false};

        return new LambdaCommand()
                .named(continuous ? "smartFeedContinuous" : "smartFeed")
                .requires(spindexer)
                .requires(intake)
                .setStart(() -> {
                    state[0] = ShootState.MOVING_TO_SHOOTER;
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
                        if (feedback != null) feedback.onSpindexerEmpty();
                        return;
                    }

                    switch (state[0]) {
                        case MOVING_TO_SHOOTER:
                            if (spindexer.atPosition() && spindexer.isAtShooterPosition()) {
                                // At shooter position, wait for shooter spin-up
                                state[0] = ShootState.WAITING_FOR_SPINUP;
                            }
                            break;

                        case WAITING_FOR_SPINUP:
                            if (shooter.atSetpoint()) {
                                // Shooter at RPM, push ball with feeder servo
                                spindexer.feederUp();
                                feedTimer.reset();
                                feedTimerStarted[0] = true;
                                if (feedback != null) {
                                    feedback.onBallShot(currentBallColor[0]);
                                }
                                state[0] = ShootState.FEEDER_UP;
                            }
                            break;

                        case FEEDER_UP:
                            // Wait for feeder to push ball into shooter
                            if (feedTimer.milliseconds() >= SpindexerConstants.FEEDER_MOVE_TIME_MS) {
                                // Ball fed, bring feeder back down
                                spindexer.feederDown();
                                feedTimer.reset();
                                spindexer.markCurrentSlotEmpty();
                                state[0] = ShootState.FEEDER_DOWN;
                            }
                            break;

                        case FEEDER_DOWN:
                            // Wait for feeder to return to rest position
                            if (feedTimer.milliseconds() >= SpindexerConstants.FEEDER_MOVE_TIME_MS) {
                                feedTimerStarted[0] = false;
                                state[0] = ShootState.NEXT_BALL;
                            }
                            break;

                        case NEXT_BALL:
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
                    spindexer.feederDown();  // Always return feeder to down position
                    spindexer.stop();
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
        final ElapsedTime feedTimer = new ElapsedTime();
        final boolean[] feedTimerStarted = {false};

        return new LambdaCommand()
                .named(continuous ? "colorSortedFeedContinuous" : "colorSortedFeed")
                .requires(spindexer)
                .requires(intake)
                .setStart(() -> {
                    currentIndex[0] = 0;
                    state[0] = ShootState.MOVING_TO_SHOOTER;

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
                        if (feedback != null) feedback.onSpindexerEmpty();
                        return;
                    }

                    // Note: Color sort order is now persistent via chassis.getColorSortTag()
                    // Use the override button to change the locked tag

                    switch (state[0]) {
                        case MOVING_TO_SHOOTER:
                            if (spindexer.atPosition() && spindexer.isAtShooterPosition()) {
                                // At shooter position, wait for shooter spin-up
                                state[0] = ShootState.WAITING_FOR_SPINUP;
                            }
                            break;

                        case WAITING_FOR_SPINUP:
                            if (shooter.atSetpoint()) {
                                // Shooter at RPM, push ball with feeder servo
                                spindexer.feederUp();
                                feedTimer.reset();
                                feedTimerStarted[0] = true;
                                if (feedback != null) {
                                    feedback.onBallShot(currentBallColor[0]);
                                }
                                state[0] = ShootState.FEEDER_UP;
                            }
                            break;

                        case FEEDER_UP:
                            // Wait for feeder to push ball into shooter
                            if (feedTimer.milliseconds() >= SpindexerConstants.FEEDER_MOVE_TIME_MS) {
                                // Ball fed, bring feeder back down
                                spindexer.feederDown();
                                feedTimer.reset();
                                spindexer.markCurrentSlotEmpty();
                                currentIndex[0]++;
                                state[0] = ShootState.FEEDER_DOWN;
                            }
                            break;

                        case FEEDER_DOWN:
                            // Wait for feeder to return to rest position
                            if (feedTimer.milliseconds() >= SpindexerConstants.FEEDER_MOVE_TIME_MS) {
                                feedTimerStarted[0] = false;
                                state[0] = ShootState.NEXT_BALL;
                            }
                            break;

                        case NEXT_BALL:
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
                    spindexer.feederDown();  // Always return feeder to down position
                    spindexer.stop();
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
        final ElapsedTime feedTimer = new ElapsedTime();
        final boolean[] feedTimerStarted = {false};

        return new LambdaCommand()
                .named("customSequenceFeed")
                .requires(spindexer)
                .requires(intake)
                .setStart(() -> {
                    currentIndex[0] = 0;
                    state[0] = ShootState.MOVING_TO_SHOOTER;

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
                        if (feedback != null) feedback.onSpindexerEmpty();
                        return;
                    }

                    switch (state[0]) {
                        case MOVING_TO_SHOOTER:
                            if (spindexer.atPosition() && spindexer.isAtShooterPosition()) {
                                // At shooter position, wait for shooter spin-up
                                state[0] = ShootState.WAITING_FOR_SPINUP;
                            }
                            break;

                        case WAITING_FOR_SPINUP:
                            if (shooter.atSetpoint()) {
                                // Shooter at RPM, push ball with feeder servo
                                spindexer.feederUp();
                                feedTimer.reset();
                                feedTimerStarted[0] = true;
                                if (feedback != null) {
                                    feedback.onBallShot(currentBallColor[0]);
                                }
                                state[0] = ShootState.FEEDER_UP;
                            }
                            break;

                        case FEEDER_UP:
                            // Wait for feeder to push ball into shooter
                            if (feedTimer.milliseconds() >= SpindexerConstants.FEEDER_MOVE_TIME_MS) {
                                // Ball fed, bring feeder back down
                                spindexer.feederDown();
                                feedTimer.reset();
                                spindexer.markCurrentSlotEmpty();
                                currentIndex[0]++;
                                state[0] = ShootState.FEEDER_DOWN;
                            }
                            break;

                        case FEEDER_DOWN:
                            // Wait for feeder to return to rest position
                            if (feedTimer.milliseconds() >= SpindexerConstants.FEEDER_MOVE_TIME_MS) {
                                feedTimerStarted[0] = false;
                                state[0] = ShootState.NEXT_BALL;
                            }
                            break;

                        case NEXT_BALL:
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
                    spindexer.feederDown();  // Always return feeder to down position
                    spindexer.stop();
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

    // ========================================================================
    // NO-INTAKE SMART FEED (for emergency mode - human player feeds balls)
    // These commands do NOT require the Intake subsystem
    // ========================================================================

    /**
     * Smart feed WITHOUT intake - for emergency mode where human player feeds balls.
     * Sequence: Move to shooter pos → Wait for RPM → Feeder up → Feeder down → Next ball
     * Runs until all balls are fired.
     */
    public static Command smartFeedNoIntake(Shooter shooter, Spindexer spindexer) {
        return smartFeedNoIntakeInternal(shooter, spindexer, null, false);
    }

    /**
     * Smart feed WITHOUT intake with feedback (LED + rumble)
     */
    public static Command smartFeedNoIntake(Shooter shooter, Spindexer spindexer, RobotFeedback feedback) {
        return smartFeedNoIntakeInternal(shooter, spindexer, feedback, false);
    }

    /**
     * Continuous smart feed WITHOUT intake for TeleOp - runs until interrupted.
     */
    public static Command smartFeedNoIntakeContinuous(Shooter shooter, Spindexer spindexer) {
        return smartFeedNoIntakeInternal(shooter, spindexer, null, true);
    }

    /**
     * Continuous smart feed WITHOUT intake with feedback
     */
    public static Command smartFeedNoIntakeContinuous(Shooter shooter, Spindexer spindexer, RobotFeedback feedback) {
        return smartFeedNoIntakeInternal(shooter, spindexer, feedback, true);
    }

    /**
     * Internal smart feed implementation WITHOUT intake
     */
    private static Command smartFeedNoIntakeInternal(Shooter shooter, Spindexer spindexer,
                                                      RobotFeedback feedback, boolean continuous) {
        final ShootState[] state = {ShootState.MOVING_TO_SHOOTER};
        final BallColor[] currentBallColor = {BallColor.UNKNOWN};
        final ElapsedTime feedTimer = new ElapsedTime();
        final boolean[] feedTimerStarted = {false};

        return new LambdaCommand()
                .named(continuous ? "smartFeedNoIntakeContinuous" : "smartFeedNoIntake")
                .requires(spindexer)
                .setStart(() -> {
                    state[0] = ShootState.MOVING_TO_SHOOTER;
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
                        if (feedback != null) feedback.onSpindexerEmpty();
                        return;
                    }

                    switch (state[0]) {
                        case MOVING_TO_SHOOTER:
                            if (spindexer.atPosition() && spindexer.isAtShooterPosition()) {
                                // At shooter position, wait for shooter spin-up
                                state[0] = ShootState.WAITING_FOR_SPINUP;
                            }
                            break;

                        case WAITING_FOR_SPINUP:
                            if (shooter.atSetpoint()) {
                                // Shooter at RPM, push ball with feeder servo
                                spindexer.feederUp();
                                feedTimer.reset();
                                feedTimerStarted[0] = true;
                                if (feedback != null) {
                                    feedback.onBallShot(currentBallColor[0]);
                                }
                                state[0] = ShootState.FEEDER_UP;
                            }
                            break;

                        case FEEDER_UP:
                            // Wait for feeder to push ball into shooter
                            if (feedTimer.milliseconds() >= SpindexerConstants.FEEDER_MOVE_TIME_MS) {
                                // Ball fed, bring feeder back down
                                spindexer.feederDown();
                                feedTimer.reset();
                                spindexer.markCurrentSlotEmpty();
                                state[0] = ShootState.FEEDER_DOWN;
                            }
                            break;

                        case FEEDER_DOWN:
                            // Wait for feeder to return to rest position
                            if (feedTimer.milliseconds() >= SpindexerConstants.FEEDER_MOVE_TIME_MS) {
                                feedTimerStarted[0] = false;
                                state[0] = ShootState.NEXT_BALL;
                            }
                            break;

                        case NEXT_BALL:
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
                    spindexer.feederDown();  // Always return feeder to down position
                    spindexer.stop();
                    if (feedback != null) feedback.onShooterStop();
                })
                .setIsDone(() -> !continuous && spindexer.isEmpty())
                .setInterruptible(true);
    }

    /**
     * Mark a specific slot as loaded (for human player feeding balls)
     * Used in no-intake emergency mode when human player puts ball in spindexer
     */
    public static Command markSlotLoaded(Spindexer spindexer, int slot) {
        return new LambdaCommand()
                .named("markSlotLoaded_" + slot)
                .requires(spindexer)
                .setStart(() -> spindexer.markSlotLoaded(slot))
                .setIsDone(() -> true)
                .setInterruptible(true);
    }

    /**
     * Rotate spindexer to intake position and mark slot as loaded.
     * For human player feeding - rotates to intake position so player can insert ball.
     */
    public static Command prepareForHumanFeed(Spindexer spindexer) {
        return new LambdaCommand()
                .named("prepareForHumanFeed")
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
     * Mark the current slot at intake position as loaded (ball inserted by human player).
     * Call this after human player has inserted the ball.
     */
    public static Command confirmBallLoaded(Spindexer spindexer) {
        return new LambdaCommand()
                .named("confirmBallLoaded")
                .requires(spindexer)
                .setStart(() -> {
                    // Get current intake slot and mark it as loaded
                    int currentPos = spindexer.getCurrentPosition();
                    int slot = SpindexerConstants.getSlotForIntakePosition(currentPos);
                    if (slot >= 0 && slot < 3) {
                        spindexer.markSlotLoaded(slot);
                    }
                })
                .setIsDone(() -> true)
                .setInterruptible(true);
    }

    // ========================================================================
    // EMERGENCY SHOOTER SMART FEED (for ShooterEmergency subsystem)
    // ========================================================================

    /**
     * Continuous smart feed for EMERGENCY SHOOTER - uses isSpinning() instead of atSetpoint()
     */
    public static Command smartFeedNoIntakeContinuous(ShooterEmergency shooter, Spindexer spindexer, RobotFeedback feedback) {
        final ShootState[] state = {ShootState.MOVING_TO_SHOOTER};
        final BallColor[] currentBallColor = {BallColor.UNKNOWN};
        final ElapsedTime feedTimer = new ElapsedTime();
        final ElapsedTime spinupTimer = new ElapsedTime();
        final boolean[] feedTimerStarted = {false};

        // For emergency shooter, wait a fixed time for spinup instead of checking RPM
        final int SPINUP_WAIT_MS = 500;

        return new LambdaCommand()
                .named("smartFeedEmergencyContinuous")
                .requires(spindexer)
                .setStart(() -> {
                    state[0] = ShootState.MOVING_TO_SHOOTER;
                    spindexer.goToNextShooterPosition();
                    spinupTimer.reset();

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
                        if (feedback != null) feedback.onSpindexerEmpty();
                        return;
                    }

                    switch (state[0]) {
                        case MOVING_TO_SHOOTER:
                            if (spindexer.atPosition() && spindexer.isAtShooterPosition()) {
                                // At shooter position, wait for shooter spin-up
                                state[0] = ShootState.WAITING_FOR_SPINUP;
                                spinupTimer.reset();
                            }
                            break;

                        case WAITING_FOR_SPINUP:
                            // Emergency mode: wait fixed time OR check if spinning
                            if (spinupTimer.milliseconds() >= SPINUP_WAIT_MS && shooter.isSpinning()) {
                                // Shooter ready, push ball with feeder servo
                                spindexer.feederUp();
                                feedTimer.reset();
                                feedTimerStarted[0] = true;
                                if (feedback != null) {
                                    feedback.onBallShot(currentBallColor[0]);
                                }
                                state[0] = ShootState.FEEDER_UP;
                            }
                            break;

                        case FEEDER_UP:
                            // Wait for feeder to push ball into shooter
                            if (feedTimer.milliseconds() >= SpindexerConstants.FEEDER_MOVE_TIME_MS) {
                                // Ball fed, bring feeder back down
                                spindexer.feederDown();
                                feedTimer.reset();
                                spindexer.markCurrentSlotEmpty();
                                state[0] = ShootState.FEEDER_DOWN;
                            }
                            break;

                        case FEEDER_DOWN:
                            // Wait for feeder to return
                            if (feedTimer.milliseconds() >= SpindexerConstants.FEEDER_MOVE_TIME_MS) {
                                // Move to next ball
                                state[0] = ShootState.NEXT_BALL;
                            }
                            break;

                        case NEXT_BALL:
                            // Move spindexer to next loaded position
                            spindexer.goToNextShooterPosition();

                            // Get color of next ball
                            int slot = getNextLoadedSlot(spindexer);
                            if (slot >= 0) {
                                currentBallColor[0] = spindexer.getBallColor(slot);
                                if (feedback != null) {
                                    feedback.onShooterSpinUp(currentBallColor[0]);
                                }
                            }

                            state[0] = ShootState.MOVING_TO_SHOOTER;
                            break;
                    }

                    // Telemetry
                    try {
                        ActiveOpMode.telemetry().addData("Emergency Feed State", state[0].name());
                    } catch (Exception e) { }
                })
                .setStop(interrupted -> {
                    spindexer.stop();
                    spindexer.feederDown();
                })
                .setIsDone(() -> false)
                .setInterruptible(true);
    }
}
