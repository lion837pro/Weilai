package org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.LED.RobotFeedback;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer.Spindexer;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;

public class IntakeCommands {

    // ========================================================================
    // LOW-LEVEL INTAKE COMMANDS
    // ========================================================================

    /**
     * Run intake at a specific speed
     */
    public static Command runIntake(Intake intake, double speed) {
        return new LambdaCommand()
                .named("runIntake")
                .requires(intake)
                .setStart(() -> {})
                .setUpdate(() -> intake.MoveIn(speed))
                .setStop(interrupted -> intake.MoveIn(0))
                .setIsDone(() -> true)
                .setInterruptible(true);
    }

    /**
     * Stop the intake
     */
    public static Command stopIntake(Intake intake) {
        return new LambdaCommand()
                .named("stopIntake")
                .requires(intake)
                .setStart(() -> intake.MoveIn(0))
                .setUpdate(() -> intake.MoveIn(0))
                .setIsDone(() -> true)
                .setInterruptible(true);
    }

    // ========================================================================
    // INTAKE SEQUENCES WITH SPINDEXER
    // ========================================================================

    /**
     * Smart intake with spindexer - runs intake and auto-indexes when ball detected.
     * Runs until interrupted (for TeleOp button hold).
     */
    public static Command runIntakeWithSpindexer(Spindexer spindexer, Intake intake, double intakeSpeed) {
        return runIntakeWithSpindexerInternal(spindexer, intake, intakeSpeed, null, false);
    }

    /**
     * Smart intake with spindexer and feedback (LED + rumble)
     */
    public static Command runIntakeWithSpindexer(Spindexer spindexer, Intake intake,
                                                  double intakeSpeed, RobotFeedback feedback) {
        return runIntakeWithSpindexerInternal(spindexer, intake, intakeSpeed, feedback, false);
    }

    /**
     * Smart intake that stops when spindexer is full (for auto routines).
     */
    public static Command intakeUntilFull(Spindexer spindexer, Intake intake, double intakeSpeed) {
        return runIntakeWithSpindexerInternal(spindexer, intake, intakeSpeed, null, true);
    }

    /**
     * Smart intake until full with feedback
     */
    public static Command intakeUntilFull(Spindexer spindexer, Intake intake,
                                           double intakeSpeed, RobotFeedback feedback) {
        return runIntakeWithSpindexerInternal(spindexer, intake, intakeSpeed, feedback, true);
    }

    /**
     * Internal implementation for intake with spindexer
     */
    private static Command runIntakeWithSpindexerInternal(Spindexer spindexer, Intake intake,
                                                           double intakeSpeed, RobotFeedback feedback,
                                                           boolean stopWhenFull) {
        final int[] previousBallCount = {0};

        return new LambdaCommand()
                .named(stopWhenFull ? "intakeUntilFull" : "runIntakeWithSpindexer")
                .requires(spindexer)
                .requires(intake)
                .setStart(() -> {
                    previousBallCount[0] = spindexer.getBallCount();
                    spindexer.goToNextIntakePosition();
                    if (feedback != null) {
                        feedback.onIntakeStart();
                    }
                })
                .setUpdate(() -> {
                    if (spindexer.isFull()) {
                        intake.MoveIn(0);
                        if (feedback != null) {
                            feedback.onSpindexerFull();
                        }
                        return;
                    }

                    // Keep intake running continuously while button is held
                    intake.MoveIn(intakeSpeed);

                    // Check for ball detection when at intake position
                    if (spindexer.atPosition() && spindexer.isAtIntakePosition()) {
                        int currentSlot = spindexer.getCurrentPosition() / 2;
                        if (spindexer.hasBall(currentSlot)) {
                            // Ball was just detected - provide feedback
                            int currentBallCount = spindexer.getBallCount();
                            if (currentBallCount > previousBallCount[0]) {
                                if (feedback != null) {
                                    feedback.onIntakeSuccess();
                                }
                                previousBallCount[0] = currentBallCount;
                            }
                            // Automatically move spindexer to next position after ball detected
                            spindexer.goToNextIntakePosition();
                        }
                    }
                })
                .setStop(interrupted -> {
                    intake.MoveIn(0);
                    spindexer.stop();
                    if (feedback != null) {
                        feedback.onIntakeStop();
                    }
                })
                .setIsDone(() -> stopWhenFull ? spindexer.isFull() : false)
                .setInterruptible(true);
    }
}
