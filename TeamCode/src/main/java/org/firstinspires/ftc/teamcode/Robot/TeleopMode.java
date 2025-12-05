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

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

/**
 * ═════════════════════════════════════════════════════════════════════════
 *                          SINGLE DRIVER TELEOP MODE
 * ═════════════════════════════════════════════════════════════════════════
 *
 * Single gamepad controls everything - drive, intake, shooter, spindexer
 *
 * QUICK REFERENCE:
 * ────────────────
 * Movement:
 *   - Left Stick: Drive (field-oriented holonomic)
 *   - Right Stick X: Turn
 *   - Options: Reset heading
 *
 * Intake:
 *   - A: Intake with auto-spindexer
 *   - B: Reverse intake
 *
 * Shooter:
 *   - X: Fixed 1600 RPM shoot
 *   - Y: Fixed 1800 RPM with spindexer
 *   - Right Bumper: Auto-aim color-sorted shooting (MAIN)
 *   - Right Trigger: Manual shooter power
 *   - Dpad Up: Reverse shooter (jam clear)
 *
 * Vision:
 *   - Left Bumper: Vision-assisted alignment
 *   - Dpad Left: Reset color sort tag
 *
 * Spindexer:
 *   - Left Trigger: Manual spindexer control
 *   - Dpad Down: Index forward
 *
 * ═════════════════════════════════════════════════════════════════════════
 */
@TeleOp(name = "Single Driver Mode", group = "Competition")
public class TeleopMode extends NextFTCOpMode {

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
    //                           BUTTON DECLARATIONS
    // ═════════════════════════════════════════════════════════════════════

    // Face buttons
    private Button a;
    private Button b;
    private Button x;
    private Button y;

    // Bumpers and triggers handled in default commands
    private Button right_bumper;
    private Button left_bumper;

    // D-pad
    private Button dpad_up;
    private Button dpad_down;
    private Button dpad_left;

    // System buttons
    private Button options;

    // ═════════════════════════════════════════════════════════════════════
    //                           INITIALIZATION
    // ═════════════════════════════════════════════════════════════════════

    public TeleopMode() {
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
            led = null;  // LED not configured
        }
        feedback = new RobotFeedback(led);
        feedback.setGamepads(gamepad1, gamepad2);

        // ─────────────────────────────────────────────────────────────────
        // Initialize Button Bindings
        // ─────────────────────────────────────────────────────────────────
        this.a = button(() -> gamepad1.a);
        this.b = button(() -> gamepad1.b);
        this.x = button(() -> gamepad1.x);
        this.y = button(() -> gamepad1.y);
        this.right_bumper = button(() -> gamepad1.right_bumper);
        this.left_bumper = button(() -> gamepad1.left_bumper);
        this.options = button(() -> gamepad1.options);
        this.dpad_up = button(() -> gamepad1.dpad_up);
        this.dpad_down = button(() -> gamepad1.dpad_down);
        this.dpad_left = button(() -> gamepad1.dpad_left);

        // ═════════════════════════════════════════════════════════════════
        //                        BUTTON MAPPINGS
        // ═════════════════════════════════════════════════════════════════

        // ─────────────────────────────────────────────────────────────────
        // SYSTEM CONTROLS
        // ─────────────────────────────────────────────────────────────────
        options.whenBecomesTrue(DriveCommands.resetHeading(chassis));

        dpad_left.whenBecomesTrue(new dev.nextftc.core.commands.utility.InstantCommand(
                "Reset Color Sort Tag",
                chassis::resetColorSortTag
        ));

        // ─────────────────────────────────────────────────────────────────
        // INTAKE CONTROLS
        // ─────────────────────────────────────────────────────────────────
        // A: Smart intake with spindexer auto-indexing
        a.whenBecomesTrue(IntakeCommands.runIntakeWithSpindexer(spindexer, intake, 0.6, feedback));
        a.whenBecomesFalse(SpindexerCommands.stopSpindexer(spindexer));
        a.whenBecomesFalse(IntakeCommands.stopIntake(intake));

        // B: Reverse intake (for clearing jams)
        b.whenBecomesTrue(IntakeCommands.runIntake(intake, -0.6));
        b.whenBecomesFalse(IntakeCommands.stopIntake(intake));

        // ─────────────────────────────────────────────────────────────────
        // SHOOTER CONTROLS
        // ─────────────────────────────────────────────────────────────────
        // X: Fixed RPM shooting with vibration feedback (1600 RPM)
        x.whenTrue(ShooterCommands.runShooterPID(shooter, 1600, feedback));
        x.whenBecomesFalse(ShooterCommands.stopShooter(shooter));

        // Dpad Up: Reverse shooter (for clearing jams)
        dpad_up.whenTrue(ShooterCommands.runShooterPID(shooter, -600));
        dpad_up.whenBecomesFalse(ShooterCommands.stopShooter(shooter));

        // ─────────────────────────────────────────────────────────────────
        // FULL SHOOTING SEQUENCES (with Spindexer)
        // ─────────────────────────────────────────────────────────────────
        // Right Bumper: Color-sorted auto-aim shooting (PRIMARY COMPETITION CONTROL)
        right_bumper.whenTrue(ShooterCommands.teleopShootColorSortedAutoAim(
                shooter, spindexer, intake, chassis, feedback));
        right_bumper.whenBecomesFalse(ShooterCommands.stopShooter(shooter));
        right_bumper.whenBecomesFalse(SpindexerCommands.stopSpindexer(spindexer));
        right_bumper.whenBecomesFalse(IntakeCommands.stopIntake(intake));

        // Y: Fixed RPM shooting with spindexer (1800 RPM - backup if vision fails)
        y.whenBecomesTrue(ShooterCommands.teleopShootFixedRPM(shooter, spindexer, intake, 1800, feedback));
        y.whenBecomesFalse(ShooterCommands.stopShooter(shooter));
        y.whenBecomesFalse(SpindexerCommands.stopSpindexer(spindexer));
        y.whenBecomesFalse(IntakeCommands.stopIntake(intake));

        // ─────────────────────────────────────────────────────────────────
        // VISION CONTROLS
        // ─────────────────────────────────────────────────────────────────
        // Left Bumper: Vision-assisted alignment with joystick movement
        left_bumper.whenTrue(
                DriveCommands.alignWithJoysticks(chassis,
                        () -> -gamepad1.left_stick_y,
                        () -> -gamepad1.left_stick_x)
        );

        // ─────────────────────────────────────────────────────────────────
        // SPINDEXER MANUAL CONTROLS
        // ─────────────────────────────────────────────────────────────────
        // Dpad Down: Manual index forward (for adjustments)
        dpad_down.whenBecomesTrue(SpindexerCommands.indexForward(spindexer));

        // ═════════════════════════════════════════════════════════════════
        //                        DEFAULT COMMANDS
        // ═════════════════════════════════════════════════════════════════

        // Drive: Field-oriented holonomic drive
        chassis.setDefaultCommand(
                DriveCommands.runWithJoysticks(chassis,
                        () -> -gamepad1.left_stick_y,      // Forward/back
                        () -> -gamepad1.left_stick_x,      // Strafe
                        () -> -gamepad1.right_stick_x,     // Turn
                        false));                            // Field-oriented

        // Shooter: Manual power control with right trigger
        shooter.setDefaultCommand(
                ShooterCommands.runManualShooter(shooter,
                        () -> gamepad1.right_trigger));

        // Spindexer: Manual spin with left trigger
        spindexer.setDefaultCommand(
                SpindexerCommands.manualSpin(spindexer,
                        () -> gamepad1.left_trigger - gamepad2.left_trigger));

        // Set LED to ready state
        feedback.setReady();
    }

    // ═════════════════════════════════════════════════════════════════════
    //                           LIFECYCLE METHODS
    // ═════════════════════════════════════════════════════════════════════

    @Override
    public void onWaitForStart() {
        // Optional: Home spindexer during init
        // SpindexerCommands.homeSpindexer(spindexer).invoke();
    }

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
        telemetry.addData("Override", "DPAD LEFT to reset");
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
