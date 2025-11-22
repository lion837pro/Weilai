package org.firstinspires.ftc.teamcode.Robot.robot2;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static dev.nextftc.bindings.Bindings.button;

import org.firstinspires.ftc.teamcode.Robot.DriveCommands.DriveCommands;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.Vision;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.IntakeCommands;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.ShooterCommands;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "TeleOp With IMU Chassis")
public class Teleoppp extends NextFTCOpMode {

    // Subsystems
    private final Chassiss chassis = Chassiss.INSTANCE;
    private final Vision vision = Vision.INSTANCE;
    private final Intake intake = Intake.INSTANCE;
    private final Shooter shooter = Shooter.INSTANCE;

    // Buttons
    private Button a;
    private Button b;
    private Button x;
    private Button y;
    private Button right_bumper;
    private Button left_bumper;
    private Button options;
    private Button dpad_up;
    private Button dpad_down;

    public Teleoppp() {
        // Add subsystem components
        addComponents(chassis.asCOMPONENT());
        addComponents(vision.asCOMPONENT());
        addComponents(intake.asCOMPONENT());
        addComponents(shooter.asCOMPONENT());
    }

    @Override
    public void onInit() {
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

        // ========== CHASSIS CONTROLS ==========

        // Reset heading with Options button
        options.whenBecomesTrue(DriveCommands.resetHeading(chassis));

        // Default drive command - field centric control
        chassis.setDefaultCommand(
                DriveCommands.runWithJoysticks(
                        chassis,
                        () -> -gamepad1.left_stick_y,
                        () -> -gamepad1.left_stick_x,
                        () -> -gamepad1.right_stick_x,
                        true // field-centric
                )
        );

        // Left bumper - Auto-align to AprilTag while driving
        left_bumper.whenTrue(
                DriveCommands.alignWithJoysticks(
                        chassis,
                        vision,
                        () -> -gamepad1.left_stick_y,
                        () -> -gamepad1.left_stick_x
                )
        );

        // ========== INTAKE CONTROLS ==========

        // A button - Intake forward
        a.whenBecomesTrue(IntakeCommands.runIntake(intake, 0.6));
        a.whenBecomesFalse(IntakeCommands.stopIntake(intake));

        // B button - Intake reverse
        b.whenBecomesTrue(IntakeCommands.runIntake(intake, -0.6));
        b.whenBecomesFalse(IntakeCommands.stopIntake(intake));

        // ========== SHOOTER CONTROLS ==========

        // Manual shooter control with right trigger
        shooter.setDefaultCommand(
                ShooterCommands.runManualShooter(shooter, () -> gamepad1.right_trigger)
        );

        // X button - Run shooter at fixed RPM
        x.whenTrue(ShooterCommands.runShooterPID(shooter, 2000));
        x.whenBecomesFalse(ShooterCommands.stopShooter(shooter));

        // DPad Up - Run shooter at low speed (for testing)
        dpad_up.whenTrue(ShooterCommands.runShooterPID(shooter, 1000));
        dpad_up.whenBecomesFalse(ShooterCommands.stopShooter(shooter));

        // Right bumper - Auto-aim and shoot with AprilTag
        right_bumper.whenTrue(
                ShooterCommands.shootWithAutoAim(shooter, intake, vision)
        );
        right_bumper.whenBecomesFalse(ShooterCommands.stopShooter(shooter));
        right_bumper.whenBecomesFalse(IntakeCommands.stopIntake(intake));

        // ========== ADDITIONAL FEATURES ==========

        // Y button - Toggle brake/coast mode (optional)
        // Uncomment if you want this feature
        // y.whenBecomesTrue(new InstantCommand("ToggleBrake", () -> {
        //     chassis.setBrakeMode();
        //     // Or toggle between brake and coast
        // }));
    }

    @Override
    public void onWaitForStart() {
        telemetry.addLine("=== ROBOT READY ===");
        telemetry.addLine();
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("Left Stick: Drive (Forward/Strafe)");
        telemetry.addLine("Right Stick X: Turn");
        telemetry.addLine("Left Bumper: Auto-Align to AprilTag");
        telemetry.addLine("Right Bumper: Auto-Aim & Shoot");
        telemetry.addLine("Right Trigger: Manual Shooter");
        telemetry.addLine("A: Intake Forward");
        telemetry.addLine("B: Intake Reverse");
        telemetry.addLine("X: Shooter Fixed RPM");
        telemetry.addLine("Options: Reset Heading");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // Any initialization when start is pressed
    }

    @Override
    public void onUpdate() {
        // Update bindings
        BindingManager.update();

        // Display telemetry
        telemetry.addData("Heading", "%.2f deg", chassis.getHeading().inDeg);
        telemetry.addData("Field Centric", chassis.isFieldCentric() ? "ON" : "OFF");

        if (vision.isConnected()) {
            telemetry.addData("Vision", "Connected");
            if (vision.hasTarget()) {
                telemetry.addData("Target", "ID %d | TX: %.2f",
                        vision.getLastDetectedId(), vision.getTx());
                telemetry.addData("Distance", "%.1f in", vision.getDistanceToTag());
            } else {
                telemetry.addData("Target", "None");
            }
        } else {
            telemetry.addData("Vision", "Disconnected");
        }

        telemetry.update();
    }

    @Override
    public void onStop() {
        // Reset bindings
        BindingManager.reset();

        // Stop all subsystems
        chassis.stop();
        intake.MoveIn(0);
        shooter.stop();
    }
}