package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static dev.nextftc.bindings.Bindings.button;

import org.firstinspires.ftc.teamcode.Robot.DriveCommands.DriveCommands;
import org.firstinspires.ftc.teamcode.Robot.Hardware.REV312010;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.ChassisConstants;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.SuperChassis;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.IntakeCommands;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.LED.RobotFeedback;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.ShooterCommands;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer.Spindexer;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer.SpindexerCommands;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer.SpindexerConstants;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

import java.util.ArrayList;
import java.util.List;

/**
 * ═════════════════════════════════════════════════════════════════════════
 *                          2-DRIVER TELEOP MODE
 * ═════════════════════════════════════════════════════════════════════════
 *
 * GAMEPAD 1 (DRIVER) - Movement & Scoring
 * GAMEPAD 2 (SPINDEXER OPERATOR) - Ball Management & Custom Sequences
 *
 * ═════════════════════════════════════════════════════════════════════════
 * GAMEPAD 1 (DRIVER) QUICK REFERENCE:
 * ─────────────────────────────────────────────────────────────────────────
 * Movement:
 *   Left Stick        → Drive (field-oriented holonomic)
 *   Right Stick X     → Turn
 *   Options           → Reset heading
 *
 * Intake:
 *   A Button          → Intake with auto-spindexer
 *   B Button          → Reverse intake
 *
 * Shooter:
 *   X Button          → Fixed 1600 RPM shoot (with vibration)
 *   Y Button          → Fixed 1800 RPM with spindexer
 *   Right Bumper      → Auto-aim color-sorted shooting ★ PRIMARY ★
 *   Right Trigger     → Manual shooter power
 *   Dpad Up           → Reverse shooter (jam clear)
 *
 * Vision:
 *   Left Bumper       → Vision-assisted alignment
 *   Dpad Left         → Reset color sort tag
 *
 * ═════════════════════════════════════════════════════════════════════════
 * GAMEPAD 2 (SPINDEXER OPERATOR) QUICK REFERENCE:
 * ─────────────────────────────────────────────────────────────────────────
 * Direct Slot Control:
 *   A Button          → Select/shoot slot 0
 *   B Button          → Select/shoot slot 1
 *   X Button          → Select/shoot slot 2
 *
 * Custom Sequence Programming:
 *   Left Bumper HOLD  → Enter programming mode
 *   + A/B/X           → Add slots to sequence (rumbles on each)
 *   Release LB        → Save sequence (rumbles to confirm)
 *   Y Button          → Execute programmed sequence
 *   Right Bumper      → Reset/clear sequence (rumbles to confirm)
 *
 * Manual Control:
 *   Left Trigger      → Spin backward
 *   Right Trigger     → Spin forward
 *   Dpad Up           → Home spindexer
 *   Dpad Down         → Index forward
 *   Dpad Left         → Index backward
 *   Dpad Right        → Go to intake position
 *
 * ═════════════════════════════════════════════════════════════════════════
 * CUSTOM SEQUENCE EXAMPLE:
 * ─────────────────────────────────────────────────────────────────────────
 * To shoot in order 2→0→1:
 *   1. Hold Left Bumper (GP2)
 *   2. Press X (adds slot 2, short rumble)
 *   3. Press A (adds slot 0, short rumble)
 *   4. Press B (adds slot 1, short rumble)
 *   5. Release Left Bumper (sequence saved, rumble confirms)
 *   6. Press Y to execute
 *
 * Telemetry shows: "Custom Sequence: [2 → 0 → 1]"
 * ═════════════════════════════════════════════════════════════════════════
 */
@TeleOp(name = "2 Driver Mode", group = "Competition")
public class TeleopMode2Driver extends NextFTCOpMode {

    // ═════════════════════════════════════════════════════════════════════
    //                           SUBSYSTEM INSTANCES
    // ═════════════════════════════════════════════════════════════════════

    private final SuperChassis chassis = SuperChassis.INSTANCE;
    private final Intake intake = Intake.INSTANCE;
    private final Shooter shooter = Shooter.INSTANCE;
    private final Spindexer spindexer = Spindexer.INSTANCE;

    // LED and feedback system
    private REV312010 led;
    private RobotFeedback feedback;

    // ═════════════════════════════════════════════════════════════════════
    //                       GAMEPAD 1 BUTTON DECLARATIONS
    // ═════════════════════════════════════════════════════════════════════

    private Button gp1_a;
    private Button gp1_b;
    private Button gp1_x;
    private Button gp1_y;
    private Button gp1_right_bumper;
    private Button gp1_left_bumper;
    private Button gp1_options;
    private Button gp1_dpad_up;
    private Button gp1_dpad_down;
    private Button gp1_dpad_left;

    // ═════════════════════════════════════════════════════════════════════
    //                       GAMEPAD 2 BUTTON DECLARATIONS
    // ═════════════════════════════════════════════════════════════════════

    private Button gp2_a;
    private Button gp2_b;
    private Button gp2_x;
    private Button gp2_y;
    private Button gp2_right_bumper;
    private Button gp2_left_bumper;
    private Button gp2_dpad_up;
    private Button gp2_dpad_down;
    private Button gp2_dpad_left;
    private Button gp2_dpad_right;

    // ═════════════════════════════════════════════════════════════════════
    //                       CUSTOM SEQUENCE STATE
    // ═════════════════════════════════════════════════════════════════════

    private final List<Integer> customSequence = new ArrayList<>();
    private boolean programmingMode = false;

    // ═════════════════════════════════════════════════════════════════════
    //                           INITIALIZATION
    // ═════════════════════════════════════════════════════════════════════

    public TeleopMode2Driver() {
        addComponents(new PedroComponent(ChassisConstants::buildPedroPathing));
        addComponents(chassis.asCOMPONENT());
        addComponents(intake.asCOMPONENT());
        addComponents(shooter.asCOMPONENT());
        addComponents(spindexer.asCOMPONENT());
    }

    @Override
    public void onInit() {
        // ─────────────────────────────────────────────────────────────────
        // Initialize Feedback System (LED + Rumble)
        // ─────────────────────────────────────────────────────────────────
        try {
            led = new REV312010();
        } catch (Exception e) {
            led = null;
        }
        feedback = new RobotFeedback(led);
        feedback.setGamepads(gamepad1, gamepad2);

        // ─────────────────────────────────────────────────────────────────
        // Initialize Gamepad 1 Buttons (DRIVER)
        // ─────────────────────────────────────────────────────────────────
        this.gp1_a = button(() -> gamepad1.a);
        this.gp1_b = button(() -> gamepad1.b);
        this.gp1_x = button(() -> gamepad1.x);
        this.gp1_y = button(() -> gamepad1.y);
        this.gp1_right_bumper = button(() -> gamepad1.right_bumper);
        this.gp1_left_bumper = button(() -> gamepad1.left_bumper);
        this.gp1_options = button(() -> gamepad1.options);
        this.gp1_dpad_up = button(() -> gamepad1.dpad_up);
        this.gp1_dpad_down = button(() -> gamepad1.dpad_down);
        this.gp1_dpad_left = button(() -> gamepad1.dpad_left);

        // ─────────────────────────────────────────────────────────────────
        // Initialize Gamepad 2 Buttons (SPINDEXER OPERATOR)
        // ─────────────────────────────────────────────────────────────────
        this.gp2_a = button(() -> gamepad2.a);
        this.gp2_b = button(() -> gamepad2.b);
        this.gp2_x = button(() -> gamepad2.x);
        this.gp2_y = button(() -> gamepad2.y);
        this.gp2_right_bumper = button(() -> gamepad2.right_bumper);
        this.gp2_left_bumper = button(() -> gamepad2.left_bumper);
        this.gp2_dpad_up = button(() -> gamepad2.dpad_up);
        this.gp2_dpad_down = button(() -> gamepad2.dpad_down);
        this.gp2_dpad_left = button(() -> gamepad2.dpad_left);
        this.gp2_dpad_right = button(() -> gamepad2.dpad_right);

        // ═════════════════════════════════════════════════════════════════
        //                  GAMEPAD 1 CONTROLS (DRIVER)
        // ═════════════════════════════════════════════════════════════════

        // ─────────────────────────────────────────────────────────────────
        // System Controls
        // ─────────────────────────────────────────────────────────────────
        gp1_options.whenBecomesTrue(DriveCommands.resetHeading(chassis));

        gp1_dpad_left.whenBecomesTrue(new InstantCommand(
                "Reset Color Sort Tag",
                chassis::resetColorSortTag
        ));

        // ─────────────────────────────────────────────────────────────────
        // Intake Controls
        // ─────────────────────────────────────────────────────────────────
        gp1_a.whenBecomesTrue(IntakeCommands.runIntakeWithSpindexer(spindexer, intake, 0.6, feedback));
        gp1_a.whenBecomesFalse(SpindexerCommands.stopSpindexer(spindexer));
        gp1_a.whenBecomesFalse(IntakeCommands.stopIntake(intake));

        gp1_b.whenBecomesTrue(IntakeCommands.runIntake(intake, -0.6));
        gp1_b.whenBecomesFalse(IntakeCommands.stopIntake(intake));

        // ─────────────────────────────────────────────────────────────────
        // Shooter Controls
        // ─────────────────────────────────────────────────────────────────
        gp1_x.whenTrue(ShooterCommands.runShooterPID(shooter, 1600, feedback));
        gp1_x.whenBecomesFalse(ShooterCommands.stopShooter(shooter));

        gp1_dpad_up.whenTrue(ShooterCommands.runShooterPID(shooter, -600));
        gp1_dpad_up.whenBecomesFalse(ShooterCommands.stopShooter(shooter));

        // ─────────────────────────────────────────────────────────────────
        // Full Shooting Sequences
        // ─────────────────────────────────────────────────────────────────
        gp1_right_bumper.whenTrue(ShooterCommands.teleopShootColorSortedAutoAim(
                shooter, spindexer, intake, chassis, feedback));
        gp1_right_bumper.whenBecomesFalse(ShooterCommands.stopShooter(shooter));
        gp1_right_bumper.whenBecomesFalse(SpindexerCommands.stopSpindexer(spindexer));
        gp1_right_bumper.whenBecomesFalse(IntakeCommands.stopIntake(intake));

        gp1_y.whenBecomesTrue(ShooterCommands.teleopShootFixedRPM(shooter, spindexer, intake, 1800, feedback));
        gp1_y.whenBecomesFalse(ShooterCommands.stopShooter(shooter));
        gp1_y.whenBecomesFalse(SpindexerCommands.stopSpindexer(spindexer));
        gp1_y.whenBecomesFalse(IntakeCommands.stopIntake(intake));

        // ─────────────────────────────────────────────────────────────────
        // Vision Controls
        // ─────────────────────────────────────────────────────────────────
        gp1_left_bumper.whenTrue(
                DriveCommands.alignWithJoysticks(chassis,
                        () -> -gamepad1.left_stick_y,
                        () -> -gamepad1.left_stick_x)
        );

        // ─────────────────────────────────────────────────────────────────
        // Default Commands (Gamepad 1)
        // ─────────────────────────────────────────────────────────────────
        chassis.setDefaultCommand(
                DriveCommands.runWithJoysticks(chassis,
                        () -> -gamepad1.left_stick_y,
                        () -> -gamepad1.left_stick_x,
                        () -> -gamepad1.right_stick_x,
                        false));

        shooter.setDefaultCommand(
                ShooterCommands.runManualShooter(shooter,
                        () -> gamepad1.right_trigger));

        // ═════════════════════════════════════════════════════════════════
        //            GAMEPAD 2 CONTROLS (SPINDEXER OPERATOR)
        // ═════════════════════════════════════════════════════════════════

        // ─────────────────────────────────────────────────────────────────
        // Programming Mode Toggle
        // ─────────────────────────────────────────────────────────────────
        gp2_left_bumper.whenBecomesTrue(new InstantCommand("Enter Programming Mode", () -> {
            programmingMode = true;
            gamepad2.rumble(200); // Rumble to confirm entry
        }));

        gp2_left_bumper.whenBecomesFalse(new InstantCommand("Exit Programming Mode", () -> {
            programmingMode = false;
            if (!customSequence.isEmpty()) {
                gamepad2.rumble(100); // Rumble to confirm sequence saved
            }
        }));

        // ─────────────────────────────────────────────────────────────────
        // Slot Selection / Programming
        // ─────────────────────────────────────────────────────────────────
        gp2_a.whenBecomesTrue(new InstantCommand("Slot 0 Action", () -> {
            if (programmingMode && gamepad2.left_bumper) {
                customSequence.add(0);
                gamepad2.rumble(50); // Quick tap confirmation
            } else {
                spindexer.goToPosition(SpindexerConstants.getShooterPosition(0));
            }
        }));

        gp2_b.whenBecomesTrue(new InstantCommand("Slot 1 Action", () -> {
            if (programmingMode && gamepad2.left_bumper) {
                customSequence.add(1);
                gamepad2.rumble(50);
            } else {
                spindexer.goToPosition(SpindexerConstants.getShooterPosition(1));
            }
        }));

        gp2_x.whenBecomesTrue(new InstantCommand("Slot 2 Action", () -> {
            if (programmingMode && gamepad2.left_bumper) {
                customSequence.add(2);
                gamepad2.rumble(50);
            } else {
                spindexer.goToPosition(SpindexerConstants.getShooterPosition(2));
            }
        }));

        // ─────────────────────────────────────────────────────────────────
        // Sequence Execution and Reset
        // ─────────────────────────────────────────────────────────────────
        gp2_y.whenTrue(createCustomSequenceCommand());
        gp2_y.whenBecomesFalse(ShooterCommands.stopShooter(shooter));
        gp2_y.whenBecomesFalse(SpindexerCommands.stopSpindexer(spindexer));
        gp2_y.whenBecomesFalse(IntakeCommands.stopIntake(intake));

        gp2_right_bumper.whenBecomesTrue(new InstantCommand("Reset Custom Sequence", () -> {
            customSequence.clear();
            gamepad2.rumble(300); // Long rumble to confirm reset
        }));

        // ─────────────────────────────────────────────────────────────────
        // Manual Spindexer Controls
        // ─────────────────────────────────────────────────────────────────
        gp2_dpad_up.whenBecomesTrue(SpindexerCommands.homeSpindexer(spindexer));
        gp2_dpad_down.whenBecomesTrue(SpindexerCommands.indexForward(spindexer));
        gp2_dpad_left.whenBecomesTrue(SpindexerCommands.indexBackward(spindexer));
        gp2_dpad_right.whenBecomesTrue(SpindexerCommands.prepareForIntake(spindexer));

        // ─────────────────────────────────────────────────────────────────
        // Default Command (Gamepad 2)
        // ─────────────────────────────────────────────────────────────────
        spindexer.setDefaultCommand(
                SpindexerCommands.manualSpin(spindexer,
                        () -> gamepad2.left_trigger - gamepad2.right_trigger));

        // ─────────────────────────────────────────────────────────────────
        // Set LED to ready state
        // ─────────────────────────────────────────────────────────────────
        feedback.setReady();
    }

    // ═════════════════════════════════════════════════════════════════════
    //                           HELPER METHODS
    // ═════════════════════════════════════════════════════════════════════

    /**
     * Creates command to shoot custom sequence programmed by operator
     */
    private dev.nextftc.core.commands.Command createCustomSequenceCommand() {
        return ShooterCommands.teleopShootCustomSequence(
                shooter, spindexer, intake, 1800,
                customSequence, feedback
        );
    }

    /**
     * Format custom sequence for telemetry display
     */
    private String formatSequence() {
        if (customSequence.isEmpty()) {
            return "[Empty - Hold LB + A/B/X to program]";
        }
        StringBuilder sb = new StringBuilder("[");
        for (int i = 0; i < customSequence.size(); i++) {
            sb.append(customSequence.get(i));
            if (i < customSequence.size() - 1) {
                sb.append(" → ");
            }
        }
        sb.append("]");
        return sb.toString();
    }

    // ═════════════════════════════════════════════════════════════════════
    //                           LIFECYCLE METHODS
    // ═════════════════════════════════════════════════════════════════════

    @Override
    public void onWaitForStart() {}

    @Override
    public void onStartButtonPressed() {}

    @Override
    public void onUpdate() {
        BindingManager.update();

        // Update feedback for LED strobe effect
        if (feedback != null) {
            feedback.update();
        }

        // ─────────────────────────────────────────────────────────────────
        // Telemetry Display
        // ─────────────────────────────────────────────────────────────────
        telemetry.addData("═══════════════════════════", "");
        telemetry.addData("   COLOR SORT STATUS", "");
        telemetry.addData("═══════════════════════════", "");
        telemetry.addData("Mode", chassis.getColorSortModeString());
        telemetry.addData("Override", "GP1: DPAD LEFT");
        telemetry.addData("", "");

        telemetry.addData("═══════════════════════════", "");
        telemetry.addData("   SPINDEXER OPERATOR", "");
        telemetry.addData("═══════════════════════════", "");
        telemetry.addData("Programming", programmingMode ? "★ ACTIVE ★" : "Inactive");
        telemetry.addData("Sequence", formatSequence());
        telemetry.addData("Controls", "LB+A/B/X=Program | RB=Reset | Y=Execute");
        telemetry.addData("", "");
    }

    @Override
    public void onStop() {
        BindingManager.reset();

        // Turn off LED when stopping
        if (feedback != null) {
            feedback.setIdle();
        }
    }
}
