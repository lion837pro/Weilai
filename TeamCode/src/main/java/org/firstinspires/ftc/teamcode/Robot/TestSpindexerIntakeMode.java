package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static dev.nextftc.bindings.Bindings.button;

import org.firstinspires.ftc.teamcode.Robot.Hardware.REV312010;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.IntakeCommands;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.LED.RobotFeedback;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer.Spindexer;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer.SpindexerCommands;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.ftc.NextFTCOpMode;

/**
 * TEST MODE - SPINDEXER + INTAKE
 *
 * A minimal teleop for testing spindexer and intake in isolation.
 *
 * Controls:
 * - A: Run Intake (hold)
 * - B: Reverse Intake (hold)
 * - X: Index Forward (one position)
 * - Y: Index Backward (one position)
 * - DPad Up: Home Spindexer
 * - DPad Down: Go to next intake position
 * - DPad Left: Go to next shooter position
 * - DPad Right: Clear all ball data
 * - LT: Manual spindexer spin (proportional)
 * - RT: Manual spindexer spin reverse (proportional)
 * - LB: Go to position 0
 * - RB: Go to position 3
 */
@TeleOp(name = "Test: Spindexer + Intake", group = "Testing")
public class TestSpindexerIntakeMode extends NextFTCOpMode {

    // Subsystems
    private final Intake intake = Intake.INSTANCE;
    private final Spindexer spindexer = Spindexer.INSTANCE;
    private REV312010 led;
    private RobotFeedback feedback;

    // Buttons
    private Button a, b, x, y;
    private Button dpad_up, dpad_down, dpad_left, dpad_right;
    private Button left_bumper, right_bumper;

    // Constructor
    public TestSpindexerIntakeMode() {
        addComponents(intake.asCOMPONENT());
        addComponents(spindexer.asCOMPONENT());
    }

    @Override
    public void onInit() {
        // Initialize feedback system
        try {
            led = new REV312010();
        } catch (Exception e) {
            led = null;
        }
        feedback = new RobotFeedback(led);
        feedback.setGamepads(gamepad1, gamepad2);

        // Initialize buttons
        this.a = button(() -> gamepad1.a);
        this.b = button(() -> gamepad1.b);
        this.x = button(() -> gamepad1.x);
        this.y = button(() -> gamepad1.y);
        this.dpad_up = button(() -> gamepad1.dpad_up);
        this.dpad_down = button(() -> gamepad1.dpad_down);
        this.dpad_left = button(() -> gamepad1.dpad_left);
        this.dpad_right = button(() -> gamepad1.dpad_right);
        this.left_bumper = button(() -> gamepad1.left_bumper);
        this.right_bumper = button(() -> gamepad1.right_bumper);

        // Intake controls
        a.whenBecomesTrue(IntakeCommands.runIntakeWithSpindexer(spindexer, intake, 0.7, feedback));
        a.whenBecomesFalse(IntakeCommands.stopIntakeWithSpindexer(spindexer, intake));

        b.whenBecomesTrue(IntakeCommands.runIntake(intake, -0.7));
        b.whenBecomesFalse(IntakeCommands.stopIntake(intake));

        // Spindexer position controls
        x.whenBecomesTrue(SpindexerCommands.indexForward(spindexer));
        y.whenBecomesTrue(SpindexerCommands.indexBackward(spindexer));

        // DPad controls
        dpad_up.whenBecomesTrue(SpindexerCommands.homeSpindexer(spindexer));
        dpad_down.whenBecomesTrue(SpindexerCommands.goToNextIntakePosition(spindexer));
        dpad_left.whenBecomesTrue(SpindexerCommands.goToNextShooterPosition(spindexer));
        dpad_right.whenBecomesTrue(new dev.nextftc.core.commands.utility.InstantCommand(
                "Clear Ball Data", () -> {
            for (int i = 0; i < 3; i++) {
                spindexer.setBallLoaded(i, false);
            }
        }));

        // Bumper preset positions
        left_bumper.whenBecomesTrue(SpindexerCommands.goToPosition(spindexer, 0));
        right_bumper.whenBecomesTrue(SpindexerCommands.goToPosition(spindexer, 3));

        // Manual spindexer control (default command)
        spindexer.setDefaultCommand(SpindexerCommands.manualSpin(spindexer,
                () -> gamepad1.right_trigger - gamepad1.left_trigger));

        telemetry.addLine("=== SPINDEXER + INTAKE TEST ===");
        telemetry.addLine("A: Intake | B: Reverse");
        telemetry.addLine("X: Index Fwd | Y: Index Back");
        telemetry.addLine("DPad Up: Home | Down: Next Intake");
        telemetry.addLine("DPad Left: Next Shooter | Right: Clear");
        telemetry.addLine("LB: Pos 0 | RB: Pos 3");
        telemetry.addLine("Triggers: Manual Spin");
        telemetry.update();
    }

    @Override
    public void onWaitForStart() {
        // Auto-home spindexer during init
        telemetry.addData("Spindexer", "Starting homing...");
        telemetry.update();

        spindexer.startHoming();

        long startTime = System.currentTimeMillis();
        long timeout = 5000;

        while (!spindexer.isAtHome() && (System.currentTimeMillis() - startTime) < timeout) {
            telemetry.addData("Homing", "Time: %dms", System.currentTimeMillis() - startTime);
            telemetry.addData("Limit", spindexer.getLimitSwitchRawState() ? "HIGH" : "LOW");
            telemetry.update();

            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                break;
            }
        }

        if (spindexer.isAtHome()) {
            spindexer.finishHoming();
            telemetry.addData("Spindexer", "HOMED");
        } else {
            spindexer.stop();
            telemetry.addData("Spindexer", "TIMEOUT - Check limit switch");
        }
        telemetry.update();

        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {}
    }

    @Override
    public void onStartButtonPressed() {}

    @Override
    public void onUpdate() {
        BindingManager.update();
        if (feedback != null) feedback.update();

        // Telemetry
        telemetry.addData("--- SPINDEXER TEST ---", "");
        telemetry.addData("Position", "%d (%s)",
                spindexer.getCurrentPosition(),
                spindexer.isAtIntakePosition() ? "INTAKE" : "SHOOTER");
        telemetry.addData("Encoder", "%.1f ticks", spindexer.getCurrentTicks());
        telemetry.addData("At Position", spindexer.atPosition() ? "YES" : "NO");
        telemetry.addData("Limit Switch", spindexer.isAtHome() ? "TRIGGERED" : "Open");

        // Ball status
        String slot0 = spindexer.hasBall(0) ? spindexer.getBallColor(0).toString().charAt(0) + "" : "O";
        String slot1 = spindexer.hasBall(1) ? spindexer.getBallColor(1).toString().charAt(0) + "" : "O";
        String slot2 = spindexer.hasBall(2) ? spindexer.getBallColor(2).toString().charAt(0) + "" : "O";
        telemetry.addData("Balls", "[%s %s %s] = %d", slot0, slot1, slot2, spindexer.getBallCount());
        telemetry.addData("Full", spindexer.isFull() ? "YES" : "NO");

        telemetry.addLine("");
        telemetry.addData("--- INTAKE TEST ---", "");
        telemetry.addData("A Button", gamepad1.a ? "ACTIVE" : "Released");
        telemetry.addData("B Button", gamepad1.b ? "ACTIVE" : "Released");

        telemetry.addLine("");
        telemetry.addData("Feedback", feedback != null ? feedback.getLastEvent() : "None");
        telemetry.update();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
        intake.MoveIn(0);
        spindexer.stop();
        if (feedback != null) feedback.setIdle();
    }
}
