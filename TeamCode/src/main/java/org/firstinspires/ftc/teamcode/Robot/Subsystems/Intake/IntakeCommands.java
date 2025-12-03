package org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;

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
                .setIsDone(() -> false)
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
                .setIsDone(() -> false)
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
     * Smart intake that stops when spindexer is full (for auto routines).
     */
    public static Command intakeUntilFull(Spindexer spindexer, Intake intake, double intakeSpeed) {
        return new LambdaCommand()
                .named("intakeUntilFull")
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
                .setIsDone(() -> spindexer.isFull())
                .setInterruptible(true);
    }
}
