package org.firstinspires.ftc.teamcode.Robot.Emergencia;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static dev.nextftc.bindings.Bindings.button;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.IntakeCommands;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.ShooterCommands;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.ftc.NextFTCOpMode;

/**
 * Simple TeleOp mode that doesn't use Pedro Pathing
 * Uses a basic mecanum drive with IMU-based field-centric control
 */
@TeleOp(name = "Simple TeleOp (No Pedro)", group = "Simple")
public class SimpleTeleOp extends NextFTCOpMode {

    // Subsystems
    private final SimpleMecanumChassis chassis = SimpleMecanumChassis.INSTANCE;
    private final SimpleVision vision = SimpleVision.INSTANCE;
    private final Intake intake = Intake.INSTANCE;
    private final Shooter shooter = Shooter.INSTANCE;

    // Control buttons
    private Button buttonA;
    private Button buttonB;
    private Button buttonX;
    private Button buttonY;
    private Button rightBumper;
    private Button leftBumper;
    private Button rightTriggerButton;
    private Button leftTriggerButton;
    private Button dpadUp;
    private Button dpadDown;
    private Button dpadLeft;
    private Button dpadRight;
    private Button optionsButton;
    private Button shareButton;

    public SimpleTeleOp() {
        // Register subsystems with NextFTC
        addComponents(chassis.asCOMPONENT());
        addComponents(vision.asCOMPONENT());
        addComponents(intake.asCOMPONENT());
        addComponents(shooter.asCOMPONENT());
    }

    @Override
    public void onInit() {
        // Initialize control buttons
        buttonA = button(() -> gamepad1.a);
        buttonB = button(() -> gamepad1.b);
        buttonX = button(() -> gamepad1.x);
        buttonY = button(() -> gamepad1.y);
        rightBumper = button(() -> gamepad1.right_bumper);
        leftBumper = button(() -> gamepad1.left_bumper);
        rightTriggerButton = button(() -> gamepad1.right_trigger > 0.5);
        leftTriggerButton = button(() -> gamepad1.left_trigger > 0.5);
        dpadUp = button(() -> gamepad1.dpad_up);
        dpadDown = button(() -> gamepad1.dpad_down);
        dpadLeft = button(() -> gamepad1.dpad_left);
        dpadRight = button(() -> gamepad1.dpad_right);
        optionsButton = button(() -> gamepad1.options);
        shareButton = button(() -> gamepad1.share);

        // ========== CHASSIS CONTROLS ==========

        // Default drive - field-centric with joysticks
        chassis.setDefaultCommand(
                SimpleDriveCommands.driveWithJoysticks(
                        chassis,
                        () -> -gamepad1.left_stick_y,   // Forward (inverted for intuitive control)
                        () -> -gamepad1.left_stick_x,   // Strafe (inverted for intuitive control)
                        () -> gamepad1.right_stick_x   // Turn (inverted for intuitive control)
                )
        );

        // Options button - Reset heading
        optionsButton.whenBecomesTrue(SimpleDriveCommands.resetHeading(chassis));

        // Share button - Toggle field-centric mode
        shareButton.whenBecomesTrue(SimpleDriveCommands.toggleFieldCentric(chassis));

        // Left trigger - Slow mode (hold for precision driving)
        leftTriggerButton.whenBecomesTrue(SimpleDriveCommands.enableSlowMode(chassis));
        leftTriggerButton.whenBecomesFalse(SimpleDriveCommands.enableNormalMode(chassis));

        // Left bumper - Auto-align drive (hold to drive with automatic AprilTag alignment)
        leftBumper.whenTrue(
                SimpleDriveCommands.alignToTag(
                        chassis,
                        vision
                )
        );

        // ========== INTAKE CONTROLS ==========

        // A button - Run intake forward (hold)
        buttonA.whenBecomesTrue(IntakeCommands.runIntake(intake, 0.7));
        buttonA.whenBecomesFalse(IntakeCommands.stopIntake(intake));

        // B button - Run intake reverse/eject (hold)
        buttonB.whenBecomesTrue(IntakeCommands.runIntake(intake, -0.7));
        buttonB.whenBecomesFalse(IntakeCommands.stopIntake(intake));

        // ========== SHOOTER CONTROLS ==========

        // Manual shooter control with right trigger (variable speed)
        shooter.setDefaultCommand(
                ShooterCommands.runManualShooter(
                        shooter,
                        () -> gamepad1.right_trigger * 0.9  // Scale down max power
                )
        );

        // X button - Run shooter at medium speed (2000 RPM)
        buttonX.whenTrue(ShooterCommands.runShooterPID(shooter, 2000));
        buttonX.whenBecomesFalse(ShooterCommands.stopShooter(shooter));

        // Y button - Run shooter at high speed (3000 RPM)
        buttonY.whenTrue(ShooterCommands.runShooterPID(shooter, 3000));
        buttonY.whenBecomesFalse(ShooterCommands.stopShooter(shooter));

        // DPad Up - Run shooter at low speed (1000 RPM)
        dpadUp.whenTrue(ShooterCommands.runShooterPID(shooter, 1000));
        dpadUp.whenBecomesFalse(ShooterCommands.stopShooter(shooter));

        // DPad Down - Reverse shooter slowly (for unjamming)
        dpadDown.whenTrue(ShooterCommands.runShooterPID(shooter, -500));
        dpadDown.whenBecomesFalse(ShooterCommands.stopShooter(shooter));

        // Right bumper - Auto-aim shooter based on distance
        rightBumper.whenTrue(
                new dev.nextftc.core.commands.utility.LambdaCommand()
                        .named("AutoAimShoot")
                        .requires(shooter)
                        .setUpdate(() -> {
                            if (vision.hasTarget()) {
                                double targetRPM = vision.getRecommendedRPM();
                                double targetTPS = org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.ShooterConstants.rpmToTicksPerSecond(targetRPM);
                                shooter.toVelocity(targetTPS);

                                telemetry.addData("Auto-Aim", "Active");
                                telemetry.addData("Target RPM", "%.0f", targetRPM);
                                telemetry.addData("Distance", "%.1f in", vision.calculateDistance());
                            } else {
                                // Default RPM if no target
                                shooter.toVelocity(
                                        org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.ShooterConstants.rpmToTicksPerSecond(1600)
                                );
                                telemetry.addData("Auto-Aim", "No Target - Using Default");
                            }
                        })
                        .setStop(interrupted -> shooter.stop())
                        .setIsDone(() -> false)
                        .setInterruptible(true)
        );
        rightBumper.whenBecomesFalse(ShooterCommands.stopShooter(shooter));

        // ========== AUTONOMOUS-STYLE COMMANDS (D-Pad) ==========

        // DPad Left - Turn 90 degrees left
        dpadLeft.whenBecomesTrue(SimpleDriveCommands.turnBy(chassis, -90));

        // DPad Right - Turn 90 degrees right
        dpadRight.whenBecomesTrue(SimpleDriveCommands.turnBy(chassis, 90));
    }

    @Override
    public void onWaitForStart() {
        telemetry.addLine("=== SIMPLE TELEOP (NO PEDRO) ===");
        telemetry.addLine();
        telemetry.addLine("DRIVE CONTROLS:");
        telemetry.addLine("  Left Stick: Move (Forward/Strafe)");
        telemetry.addLine("  Right Stick X: Turn");
        telemetry.addLine("  Left Trigger: Slow Mode (Hold)");
        telemetry.addLine("  Left Bumper: Auto-Align Drive (Hold)");
        telemetry.addLine("  Options: Reset Heading");
        telemetry.addLine("  Share: Toggle Field-Centric");
        telemetry.addLine();
        telemetry.addLine("MECHANISM CONTROLS:");
        telemetry.addLine("  A: Intake In");
        telemetry.addLine("  B: Intake Out");
        telemetry.addLine("  Right Trigger: Manual Shooter");
        telemetry.addLine("  X: Shooter Medium (2000 RPM)");
        telemetry.addLine("  Y: Shooter High (3000 RPM)");
        telemetry.addLine("  Right Bumper: Auto-Aim Shooter");
        telemetry.addLine();
        telemetry.addLine("D-PAD:");
        telemetry.addLine("  Up: Shooter Low (1000 RPM)");
        telemetry.addLine("  Down: Reverse Shooter");
        telemetry.addLine("  Left: Turn 90° Left");
        telemetry.addLine("  Right: Turn 90° Right");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // Any initialization when start is pressed
        telemetry.clear();
    }

    @Override
    public void onUpdate() {
        // Update bindings
        BindingManager.update();

        // Display telemetry
        telemetry.addLine("=== ROBOT STATUS ===");

        // Chassis info
        telemetry.addData("Heading", "%.1f°", chassis.getHeading());
        telemetry.addData("Mode", chassis.isFieldCentric() ? "Field-Centric" : "Robot-Centric");
        telemetry.addData("Speed", "%.0f%%", chassis.getPowerMultiplier() * 100);

        // Vision info
        if (vision.isConnected()) {
            if (vision.hasTarget()) {
                telemetry.addData("Vision", "Tag %d Detected", vision.getCurrentTagId());
                telemetry.addData("Alignment", "TX: %.1f°", vision.getTx());
                telemetry.addData("Distance", "%.1f in", vision.calculateDistance());
            } else {
                telemetry.addData("Vision", "No Target");
            }
        } else {
            telemetry.addData("Vision", "Disconnected");
        }

        // Shooter info
        if (shooter.atSetpoint()) {
            telemetry.addData("Shooter", "READY");
        } else {
            telemetry.addData("Shooter", "Spinning up...");
        }

        // Motor powers for debugging
        double[] powers = chassis.getMotorPowers();
        telemetry.addData("Motors", "FL:%.2f FR:%.2f BL:%.2f BR:%.2f",
                powers[0], powers[1], powers[2], powers[3]);

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

        telemetry.addLine("TeleOp Stopped");
        telemetry.update();
    }
}