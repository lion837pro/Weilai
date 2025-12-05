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
 * SINGLE DRIVER TELEOP MODE
 * Controls: A=Intake | B=Reverse | X=1600RPM | Y=1800RPM+Spindexer | RB=AutoAim(MAIN)
 *          LB=VisionAlign | RT=ManualShooter | LT=ManualSpindexer | Options=ResetHeading
 *          DpadLeft=ResetColorTag | DpadUp=ReverseShooter | DpadDown=IndexForward
 */
@TeleOp(name = "Single Driver Mode", group = "Competition")
public class TeleopMode extends NextFTCOpMode {

    // Subsystems
    private final SuperChassis chassis = SuperChassis.INSTANCE;
    private final Intake intake = Intake.INSTANCE;
    private final Shooter shooter = Shooter.INSTANCE;
    private final Spindexer spindexer = Spindexer.INSTANCE;
    private REV312010 led;
    private RobotFeedback feedback;

    // Buttons
    private Button a, b, x, y;
    private Button right_bumper, left_bumper;
    private Button dpad_up, dpad_down, dpad_left;
    private Button options;

    // Constructor

    public TeleopMode() {
        addComponents(new PedroComponent(ChassisConstants::buildPedroPathing));
        addComponents(chassis.asCOMPONENT());
        addComponents(intake.asCOMPONENT());
        addComponents(shooter.asCOMPONENT());
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
        this.right_bumper = button(() -> gamepad1.right_bumper);
        this.left_bumper = button(() -> gamepad1.left_bumper);
        this.options = button(() -> gamepad1.options);
        this.dpad_up = button(() -> gamepad1.dpad_up);
        this.dpad_down = button(() -> gamepad1.dpad_down);
        this.dpad_left = button(() -> gamepad1.dpad_left);

        // System controls
        options.whenBecomesTrue(DriveCommands.resetHeading(chassis));
        dpad_left.whenBecomesTrue(new dev.nextftc.core.commands.utility.InstantCommand(
                "Reset Color Sort Tag", chassis::resetColorSortTag));

        // Intake controls
        a.whenBecomesTrue(IntakeCommands.runIntakeWithSpindexer(spindexer, intake, 0.6, feedback));
        a.whenBecomesFalse(SpindexerCommands.stopSpindexer(spindexer));
        a.whenBecomesFalse(IntakeCommands.stopIntake(intake));
        b.whenBecomesTrue(IntakeCommands.runIntake(intake, -0.6));
        b.whenBecomesFalse(IntakeCommands.stopIntake(intake));

        // Shooter controls (standalone)
        x.whenTrue(ShooterCommands.runShooterPID(shooter, 1600, feedback));
        x.whenBecomesFalse(ShooterCommands.stopShooter(shooter));
        dpad_up.whenTrue(ShooterCommands.runShooterPID(shooter, -600));
        dpad_up.whenBecomesFalse(ShooterCommands.stopShooter(shooter));

        // Full shooting sequences (PRIMARY COMPETITION CONTROLS)
        right_bumper.whenTrue(ShooterCommands.teleopShootColorSortedAutoAim(
                shooter, spindexer, intake, chassis, feedback));
        right_bumper.whenBecomesFalse(ShooterCommands.stopShooter(shooter));
        right_bumper.whenBecomesFalse(SpindexerCommands.stopSpindexer(spindexer));
        right_bumper.whenBecomesFalse(IntakeCommands.stopIntake(intake));

        y.whenBecomesTrue(ShooterCommands.teleopShootFixedRPM(shooter, spindexer, intake, 1800, feedback));
        y.whenBecomesFalse(ShooterCommands.stopShooter(shooter));
        y.whenBecomesFalse(SpindexerCommands.stopSpindexer(spindexer));
        y.whenBecomesFalse(IntakeCommands.stopIntake(intake));

        // Vision controls
        left_bumper.whenTrue(DriveCommands.alignWithJoysticks(chassis,
                () -> -gamepad1.left_stick_y, () -> -gamepad1.left_stick_x));

        // Spindexer manual controls
        dpad_down.whenBecomesTrue(SpindexerCommands.indexForward(spindexer));

        // Default commands
        chassis.setDefaultCommand(DriveCommands.runWithJoysticks(chassis,
                () -> -gamepad1.left_stick_y, () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x, false));
        shooter.setDefaultCommand(ShooterCommands.runManualShooter(shooter,
                () -> gamepad1.right_trigger));
        spindexer.setDefaultCommand(SpindexerCommands.manualSpin(spindexer,
                () -> gamepad1.left_trigger));

        feedback.setReady();
    }

    // Lifecycle methods

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
        if (feedback != null) feedback.update();

        // Telemetry
        telemetry.addData("Color Sort", chassis.getColorSortModeString());
        telemetry.addData("Override", "DPAD LEFT");
    }

    @Override
    public void onStop() {
        BindingManager.reset();
        if (feedback != null) feedback.setIdle();
    }
}