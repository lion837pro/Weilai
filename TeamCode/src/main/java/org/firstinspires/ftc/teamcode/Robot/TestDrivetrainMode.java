package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static dev.nextftc.bindings.Bindings.button;

import org.firstinspires.ftc.teamcode.Robot.DriveCommands.DriveCommands;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.ChassisConstants;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.SuperChassis;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

/**
 * TEST MODE - DRIVETRAIN ONLY
 *
 * A minimal teleop for testing chassis functionality in isolation.
 *
 * Controls:
 * - Left Stick: Forward/Strafe (field-oriented)
 * - Right Stick X: Rotation
 * - Options: Reset Heading
 * - A: Toggle Robot-Centric mode
 * - B: Toggle power scale (50% / 100%)
 * - X: Test drive forward
 * - Y: Test strafe right
 */
@TeleOp(name = "Test: Drivetrain Only", group = "Testing")
public class TestDrivetrainMode extends NextFTCOpMode {

    // Subsystem
    private final SuperChassis chassis = SuperChassis.INSTANCE;

    // Buttons
    private Button a, b, x, y, options;

    // State
    private boolean robotCentric = false;
    private boolean halfPower = false;

    // Constructor
    public TestDrivetrainMode() {
        addComponents(new PedroComponent(ChassisConstants::buildPedroPathing));
        addComponents(chassis.asCOMPONENT());
    }

    @Override
    public void onInit() {
        // Initialize buttons
        this.a = button(() -> gamepad1.a);
        this.b = button(() -> gamepad1.b);
        this.x = button(() -> gamepad1.x);
        this.y = button(() -> gamepad1.y);
        this.options = button(() -> gamepad1.options);

        // System controls
        options.whenBecomesTrue(DriveCommands.resetHeading(chassis));

        // Toggle robot-centric mode
        a.whenBecomesTrue(new dev.nextftc.core.commands.utility.InstantCommand(
                "Toggle Robot Centric", () -> {
            robotCentric = !robotCentric;
        }));

        // Toggle half power
        b.whenBecomesTrue(new dev.nextftc.core.commands.utility.InstantCommand(
                "Toggle Power", () -> {
            halfPower = !halfPower;
        }));

        // Set default drive command
        chassis.setDefaultCommand(DriveCommands.runWithJoysticks(chassis,
                () -> {
                    double fw = -gamepad1.left_stick_y;
                    return halfPower ? fw * 0.5 : fw;
                },
                () -> {
                    double st = gamepad1.left_stick_x;
                    return halfPower ? st * 0.5 : st;
                },
                () -> {
                    double tr = gamepad1.right_stick_x;
                    return halfPower ? tr * 0.5 : tr;
                },
                robotCentric));

        telemetry.addLine("=== DRIVETRAIN TEST MODE ===");
        telemetry.addLine("Left Stick: Move");
        telemetry.addLine("Right Stick X: Rotate");
        telemetry.addLine("Options: Reset Heading");
        telemetry.addLine("A: Toggle Robot-Centric");
        telemetry.addLine("B: Toggle Half Power");
        telemetry.update();
    }

    @Override
    public void onWaitForStart() {
        telemetry.addData("Status", "Ready to test drivetrain");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {}

    @Override
    public void onUpdate() {
        BindingManager.update();

        // Re-set default command with current robotCentric state
        // (This is a workaround since the closure captures the initial value)
        double powerScale = halfPower ? 0.5 : 1.0;
        double fw = -gamepad1.left_stick_y * powerScale;
        double st = gamepad1.left_stick_x * powerScale;
        double tr = gamepad1.right_stick_x * powerScale;

        // Apply deadband
        if (Math.abs(fw) < 0.05) fw = 0;
        if (Math.abs(st) < 0.05) st = 0;
        if (Math.abs(tr) < 0.05) tr = 0;

        chassis.driveHolonomic(fw, st, tr, robotCentric);

        // Telemetry
        telemetry.addData("--- DRIVETRAIN TEST ---", "");
        telemetry.addData("Mode", robotCentric ? "ROBOT-CENTRIC" : "FIELD-CENTRIC");
        telemetry.addData("Power Scale", halfPower ? "50%" : "100%");
        telemetry.addData("Heading", "%.1f deg", Math.toDegrees(chassis.getAngle().inRad));
        telemetry.addData("Forward", "%.2f", fw);
        telemetry.addData("Strafe", "%.2f", st);
        telemetry.addData("Turn", "%.2f", tr);
        telemetry.addLine("");
        telemetry.addData("Controls", "A=Toggle Mode, B=Toggle Power");
        telemetry.addData("", "Options=Reset Heading");
        telemetry.update();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
        chassis.stop();
    }
}
