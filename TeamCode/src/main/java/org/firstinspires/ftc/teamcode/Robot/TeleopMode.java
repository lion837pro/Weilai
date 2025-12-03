package org.firstinspires.ftc.teamcode.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static dev.nextftc.bindings.Bindings.button;
import org.firstinspires.ftc.teamcode.Robot.DriveCommands.DriveCommands;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.ChassisConstants;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.SuperChassis;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.IntakeCommands;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.ShooterCommands;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer.Spindexer;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer.SpindexerCommands;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "NextFTC Teleop")
public class TeleopMode extends NextFTCOpMode {

    private final SuperChassis chassis = SuperChassis.INSTANCE;
    private final Intake intake = Intake.INSTANCE;
    private final Shooter shooter = Shooter.INSTANCE;
    private final Spindexer spindexer = Spindexer.INSTANCE;


    private  Button a;
    private  Button b;
    private Button right_bumper;
    private Button left_bumper;
    private Button x;
    private  Button options;
    private Button y;
    private Button dpad;
    private Button dpad_down;

    public TeleopMode() {
        addComponents(new PedroComponent(ChassisConstants::buildPedroPathing));
        addComponents(chassis.asCOMPONENT());
        addComponents(intake.asCOMPONENT());
        addComponents(shooter.asCOMPONENT());
        addComponents(spindexer.asCOMPONENT());
    }

    @Override
    public void onInit() {

        this.a = button(() -> gamepad1.a);
        this.b = button(() -> gamepad1.b);
        this.x = button(() -> gamepad1.x);
        this.y = button(() -> gamepad1.y);
        this.right_bumper = button(() -> gamepad1.right_bumper);
        this.left_bumper = button(() -> gamepad1.left_bumper);
        this.options = button(() -> gamepad1.options);
        this.dpad = button(() -> gamepad1.dpad_up);
        this.dpad_down = button(() -> gamepad1.dpad_down);

        options.whenBecomesTrue(DriveCommands.resetHeading(chassis));

        // ===== INTAKE WITH SPINDEXER =====
        // A button: Smart intake with spindexer auto-indexing
        a.whenBecomesTrue(SpindexerCommands.teleopIntake(spindexer, intake, 0.6));
        a.whenBecomesFalse(SpindexerCommands.stopSpindexer(spindexer));
        a.whenBecomesFalse(IntakeCommands.stopIntake(intake));

        // B button: Reverse intake (manual, no spindexer)
        b.whenBecomesTrue(IntakeCommands.runIntake(intake, -0.6));
        b.whenBecomesFalse(IntakeCommands.stopIntake(intake));

        // ===== SHOOTER CONTROLS =====
        // X button: Fixed RPM shooting (no spindexer integration for quick shots)
        x.whenTrue(ShooterCommands.runShooterPID(shooter, 1600));
        x.whenBecomesFalse(ShooterCommands.stopShooter(shooter));

        // Dpad Up: Reverse shooter (for clearing jams)
        dpad.whenTrue(ShooterCommands.runShooterPID(shooter, -600));
        dpad.whenBecomesFalse(ShooterCommands.stopShooter(shooter));

        // ===== FULL SHOOTING WITH SPINDEXER =====
        // Right bumper: Auto-aim shooting with spindexer indexing
        right_bumper.whenTrue(SpindexerCommands.teleopShoot(shooter, spindexer, intake, chassis));
        right_bumper.whenBecomesFalse(ShooterCommands.stopShooter(shooter));
        right_bumper.whenBecomesFalse(SpindexerCommands.stopSpindexer(spindexer));
        right_bumper.whenBecomesFalse(IntakeCommands.stopIntake(intake));

        // Y button: Fixed RPM shooting with spindexer (for when vision isn't available)
        y.whenBecomesTrue(SpindexerCommands.teleopShootFixedRPM(shooter, spindexer, intake, 1800));
        y.whenBecomesFalse(ShooterCommands.stopShooter(shooter));
        y.whenBecomesFalse(SpindexerCommands.stopSpindexer(spindexer));
        y.whenBecomesFalse(IntakeCommands.stopIntake(intake));

        // ===== SPINDEXER MANUAL CONTROLS =====
        // Dpad Down: Manual index forward (for adjustments)
        dpad_down.whenBecomesTrue(SpindexerCommands.indexForward(spindexer));

        // ===== DRIVE CONTROLS =====
        chassis.setDefaultCommand(
                DriveCommands.runWithJoysticks(chassis,
                        () -> gamepad1.left_stick_y,
                        () -> gamepad1.left_stick_x,
                        () -> -gamepad1.right_stick_x,
                        false));

        // Left bumper: Vision alignment with joystick movement
        left_bumper.whenTrue(
                DriveCommands.alignWithJoysticks(chassis,
                        () -> -gamepad1.left_stick_y,
                        () -> -gamepad1.left_stick_x)
        );

        // ===== DEFAULT COMMANDS =====
        shooter.setDefaultCommand(
                ShooterCommands.runManualShooter(shooter,
                        () -> gamepad1.right_trigger));

        // Spindexer manual spin with left trigger (for testing/clearing jams)
        spindexer.setDefaultCommand(
                SpindexerCommands.manualSpin(spindexer,
                        () -> gamepad1.left_trigger - gamepad2.left_trigger));
    }

    @Override
    public void onWaitForStart() {
        // Home spindexer during init (optional - can be removed if not needed)
        // SpindexerCommands.homeSpindexer(spindexer).invoke();
    }

    @Override
    public void onStartButtonPressed() {}

    @Override
    public void onUpdate() {
        BindingManager.update();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
    }
}
