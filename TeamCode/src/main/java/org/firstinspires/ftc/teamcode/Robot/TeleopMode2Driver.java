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

@TeleOp(name = "2 Driver Mode")
public class TeleopMode2Driver extends NextFTCOpMode {

    private final SuperChassis chassis = SuperChassis.INSTANCE;
    private final Intake intake = Intake.INSTANCE;
    private final Shooter shooter = Shooter.INSTANCE;
    private final Spindexer spindexer = Spindexer.INSTANCE;

    // LED and feedback system
    private REV312010 led;
    private RobotFeedback feedback;

    // Gamepad 1 buttons (Driver)
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

    // Gamepad 2 buttons (Spindexer Operator)
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

    // Custom shooting sequence state
    private final List<Integer> customSequence = new ArrayList<>();
    private boolean programmingMode = false;

    public TeleopMode2Driver() {
        addComponents(new PedroComponent(ChassisConstants::buildPedroPathing));
        addComponents(chassis.asCOMPONENT());
        addComponents(intake.asCOMPONENT());
        addComponents(shooter.asCOMPONENT());
        addComponents(spindexer.asCOMPONENT());
    }

    @Override
    public void onInit() {
        // Initialize LED (REV-31-2010) and feedback system
        try {
            led = new REV312010();
        } catch (Exception e) {
            led = null;  // LED not configured, feedback will work without it
        }
        feedback = new RobotFeedback(led);
        feedback.setGamepads(gamepad1, gamepad2);

        // ===== GAMEPAD 1 (DRIVER) =====
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

        // ===== GAMEPAD 2 (SPINDEXER OPERATOR) =====
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

        // ========================================================================
        // GAMEPAD 1 CONTROLS (DRIVER)
        // ========================================================================

        gp1_options.whenBecomesTrue(DriveCommands.resetHeading(chassis));

        // Color sort override
        gp1_dpad_left.whenBecomesTrue(new InstantCommand(
                "Reset Color Sort Tag",
                chassis::resetColorSortTag
        ));

        // INTAKE WITH SPINDEXER
        gp1_a.whenBecomesTrue(IntakeCommands.runIntakeWithSpindexer(spindexer, intake, 0.6, feedback));
        gp1_a.whenBecomesFalse(SpindexerCommands.stopSpindexer(spindexer));
        gp1_a.whenBecomesFalse(IntakeCommands.stopIntake(intake));

        // Reverse intake
        gp1_b.whenBecomesTrue(IntakeCommands.runIntake(intake, -0.6));
        gp1_b.whenBecomesFalse(IntakeCommands.stopIntake(intake));

        // SHOOTER CONTROLS
        // X button: Fixed RPM shooting with feedback
        gp1_x.whenTrue(ShooterCommands.runShooterPID(shooter, 1600, feedback));
        gp1_x.whenBecomesFalse(ShooterCommands.stopShooter(shooter));

        // Dpad Up: Reverse shooter (for clearing jams)
        gp1_dpad_up.whenTrue(ShooterCommands.runShooterPID(shooter, -600));
        gp1_dpad_up.whenBecomesFalse(ShooterCommands.stopShooter(shooter));

        // FULL SHOOTING WITH SPINDEXER
        // Right bumper: Color-sorted auto-aim shooting
        gp1_right_bumper.whenTrue(ShooterCommands.teleopShootColorSortedAutoAim(shooter, spindexer, intake, chassis, feedback));
        gp1_right_bumper.whenBecomesFalse(ShooterCommands.stopShooter(shooter));
        gp1_right_bumper.whenBecomesFalse(SpindexerCommands.stopSpindexer(spindexer));
        gp1_right_bumper.whenBecomesFalse(IntakeCommands.stopIntake(intake));

        // Y button: Fixed RPM shooting with spindexer
        gp1_y.whenBecomesTrue(ShooterCommands.teleopShootFixedRPM(shooter, spindexer, intake, 1800, feedback));
        gp1_y.whenBecomesFalse(ShooterCommands.stopShooter(shooter));
        gp1_y.whenBecomesFalse(SpindexerCommands.stopSpindexer(spindexer));
        gp1_y.whenBecomesFalse(IntakeCommands.stopIntake(intake));

        // Left bumper: Vision alignment with joystick movement
        gp1_left_bumper.whenTrue(
                DriveCommands.alignWithJoysticks(chassis,
                        () -> -gamepad1.left_stick_y,
                        () -> -gamepad1.left_stick_x)
        );

        // DRIVE CONTROLS
        chassis.setDefaultCommand(
                DriveCommands.runWithJoysticks(chassis,
                        () -> -gamepad1.left_stick_y,
                        () -> -gamepad1.left_stick_x,
                        () -> -gamepad1.right_stick_x,
                        false));

        // Shooter manual control with right trigger
        shooter.setDefaultCommand(
                ShooterCommands.runManualShooter(shooter,
                        () -> gamepad1.right_trigger));

        // ========================================================================
        // GAMEPAD 2 CONTROLS (SPINDEXER OPERATOR)
        // ========================================================================

        // LEFT BUMPER (HOLD): Enter programming mode
        // While held, A/B/X add slots to custom sequence
        gp2_left_bumper.whenBecomesTrue(new InstantCommand("Enter Programming Mode", () -> {
            programmingMode = true;
            gamepad2.rumble(200); // Short rumble to confirm
        }));
        gp2_left_bumper.whenBecomesFalse(new InstantCommand("Exit Programming Mode", () -> {
            programmingMode = false;
            if (!customSequence.isEmpty()) {
                gamepad2.rumble(100); // Short rumble to confirm sequence saved
            }
        }));

        // RIGHT BUMPER: Reset custom sequence
        gp2_right_bumper.whenBecomesTrue(new InstantCommand("Reset Custom Sequence", () -> {
            customSequence.clear();
            gamepad2.rumble(300); // Long rumble to confirm reset
        }));

        // A/B/X BUTTONS: Add slot to sequence (if programming) or shoot single slot
        gp2_a.whenBecomesTrue(new InstantCommand("Slot 0 Action", () -> {
            if (programmingMode && gamepad2.left_bumper) {
                customSequence.add(0);
                gamepad2.rumble(50); // Quick tap
            } else {
                // Shoot single slot 0
                spindexer.goToPosition(SpindexerConstants.getShooterPosition(0));
            }
        }));

        gp2_b.whenBecomesTrue(new InstantCommand("Slot 1 Action", () -> {
            if (programmingMode && gamepad2.left_bumper) {
                customSequence.add(1);
                gamepad2.rumble(50); // Quick tap
            } else {
                // Shoot single slot 1
                spindexer.goToPosition(SpindexerConstants.getShooterPosition(1));
            }
        }));

        gp2_x.whenBecomesTrue(new InstantCommand("Slot 2 Action", () -> {
            if (programmingMode && gamepad2.left_bumper) {
                customSequence.add(2);
                gamepad2.rumble(50); // Quick tap
            } else {
                // Shoot single slot 2
                spindexer.goToPosition(SpindexerConstants.getShooterPosition(2));
            }
        }));

        // Y BUTTON: Execute custom sequence
        gp2_y.whenTrue(createCustomSequenceCommand());
        gp2_y.whenBecomesFalse(ShooterCommands.stopShooter(shooter));
        gp2_y.whenBecomesFalse(SpindexerCommands.stopSpindexer(spindexer));
        gp2_y.whenBecomesFalse(IntakeCommands.stopIntake(intake));

        // DPAD CONTROLS
        // Dpad Up: Home spindexer
        gp2_dpad_up.whenBecomesTrue(SpindexerCommands.homeSpindexer(spindexer));

        // Dpad Down: Index forward
        gp2_dpad_down.whenBecomesTrue(SpindexerCommands.indexForward(spindexer));

        // Dpad Left: Index backward
        gp2_dpad_left.whenBecomesTrue(SpindexerCommands.indexBackward(spindexer));

        // Dpad Right: Go to intake position
        gp2_dpad_right.whenBecomesTrue(SpindexerCommands.prepareForIntake(spindexer));

        // Manual spindexer spin with triggers
        spindexer.setDefaultCommand(
                SpindexerCommands.manualSpin(spindexer,
                        () -> gamepad2.left_trigger - gamepad2.right_trigger));

        // Set LED to ready state
        feedback.setReady();
    }

    /**
     * Creates a command to shoot custom sequence programmed by operator
     */
    private dev.nextftc.core.commands.Command createCustomSequenceCommand() {
        return ShooterCommands.teleopShootCustomSequence(
                shooter, spindexer, intake, 1800,
                customSequence, feedback
        );
    }

    @Override
    public void onWaitForStart() {
        // Optional: Home spindexer during init
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

        // Display color sort mode
        telemetry.addData("=== COLOR SORT ===", "");
        telemetry.addData("Mode", chassis.getColorSortModeString());
        telemetry.addData("Override", "GP1: DPAD LEFT to reset");

        // Display custom sequence programming info
        telemetry.addData("", "");
        telemetry.addData("=== SPINDEXER OPERATOR (GP2) ===", "");
        telemetry.addData("Programming Mode", programmingMode ? "ACTIVE" : "Inactive");
        telemetry.addData("Custom Sequence", formatSequence());
        telemetry.addData("Controls", "LB+A/B/X: Program | RB: Reset | Y: Execute");
    }

    /**
     * Format custom sequence for telemetry display
     */
    private String formatSequence() {
        if (customSequence.isEmpty()) {
            return "[Empty]";
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

    @Override
    public void onStop() {
        BindingManager.reset();
        // Turn off LED when stopping
        if (feedback != null) {
            feedback.setIdle();
        }
    }
}
