package org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.SuperChassis;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.VisionConstants;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.VisionConstants.BallColor;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.Shooter;

import java.util.function.DoubleSupplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.ftc.ActiveOpMode;

/**
 * Commands for the Spindexer subsystem.
 * Contains spindexer-specific commands and smart feed helpers.
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

    // ========================================================================
    // COLOR-SORTED SMART FEED (uses AprilTag IDs 21, 22, 23)
    // ========================================================================

    /**
     * Smart feed with color sorting based on AprilTag.
     * Uses detected tag ID (21, 22, 23) to determine shooting order:
     * - Tag 21 (G,P,P): Shoot GREEN first from slot 0
     * - Tag 22 (P,G,P): Shoot GREEN first from slot 1
     * - Tag 23 (P,P,G): Shoot GREEN first from slot 2
     *
     * If no color sort tag is detected, falls back to normal sequential shooting.
     * Runs until all balls are fired.
     */
    public static Command smartFeedColorSorted(Shooter shooter, Spindexer spindexer, Intake intake, SuperChassis chassis) {
        final int[] targetSlotOrder = {-1, -1, -1};  // Will be populated based on tag
        final int[] currentIndex = {0};

        return new LambdaCommand()
                .named("smartFeedColorSorted")
                .requires(spindexer)
                .requires(intake)
                .setStart(() -> {
                    currentIndex[0] = 0;

                    // Determine shooting order based on detected tag
                    int tagId = chassis.getLastDetectedId();
                    int greenSlot = VisionConstants.getGreenSlotForTag(tagId);

                    if (greenSlot != -1) {
                        // Color sort tag detected - prioritize green ball position
                        // Shooting order: green slot first, then others
                        targetSlotOrder[0] = greenSlot;
                        int idx = 1;
                        for (int i = 0; i < 3; i++) {
                            if (i != greenSlot) {
                                targetSlotOrder[idx++] = i;
                            }
                        }
                        ActiveOpMode.telemetry().addData("ColorSort", "Tag %d: Green at slot %d", tagId, greenSlot);
                    } else {
                        // No color sort tag - use default order
                        targetSlotOrder[0] = 0;
                        targetSlotOrder[1] = 1;
                        targetSlotOrder[2] = 2;
                        ActiveOpMode.telemetry().addData("ColorSort", "No tag - default order");
                    }

                    // Move to first slot with a ball
                    goToNextSlotInOrder(spindexer, targetSlotOrder, currentIndex);
                })
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
                        currentIndex[0]++;
                        goToNextSlotInOrder(spindexer, targetSlotOrder, currentIndex);
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
     * Continuous color-sorted smart feed for TeleOp.
     * Continuously checks for new AprilTag detections and adjusts shooting order.
     */
    public static Command smartFeedColorSortedContinuous(Shooter shooter, Spindexer spindexer, Intake intake, SuperChassis chassis) {
        final int[] lastTagId = {-1};

        return new LambdaCommand()
                .named("smartFeedColorSortedContinuous")
                .requires(spindexer)
                .requires(intake)
                .setStart(() -> {
                    lastTagId[0] = -1;
                    // Initial positioning
                    int tagId = chassis.getLastDetectedId();
                    if (VisionConstants.isColorSortTag(tagId)) {
                        spindexer.goToShooterPositionForTag(tagId);
                    } else {
                        spindexer.goToNextShooterPosition();
                    }
                })
                .setUpdate(() -> {
                    if (spindexer.isEmpty()) {
                        intake.MoveIn(0);
                        return;
                    }

                    // Check for tag changes
                    int currentTag = chassis.getLastDetectedId();
                    if (VisionConstants.isColorSortTag(currentTag) && currentTag != lastTagId[0]) {
                        lastTagId[0] = currentTag;
                        // Tag changed - reposition if needed
                        spindexer.goToShooterPositionForTag(currentTag);
                        ActiveOpMode.telemetry().addData("ColorSort", "Switched to tag %d", currentTag);
                    }

                    if (!spindexer.atPosition() || !spindexer.isAtShooterPosition()) {
                        intake.MoveIn(0);
                        return;
                    }

                    if (shooter.atSetpoint()) {
                        intake.MoveIn(1.0);
                        spindexer.markCurrentSlotEmpty();

                        // Move to next appropriate slot
                        if (VisionConstants.isColorSortTag(currentTag)) {
                            spindexer.goToShooterPositionForTag(currentTag);
                        } else {
                            spindexer.goToNextShooterPosition();
                        }
                    } else {
                        intake.MoveIn(0);
                    }

                    // Telemetry
                    BallColor[] colors = spindexer.getAllBallColors();
                    ActiveOpMode.telemetry().addData("Balls", "[%s %s %s]",
                            colorChar(colors[0]), colorChar(colors[1]), colorChar(colors[2]));
                    ActiveOpMode.telemetry().addData("Tag", currentTag);
                })
                .setStop(interrupted -> {
                    intake.MoveIn(0);
                    spindexer.stop();
                })
                .setIsDone(() -> false)
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
}
