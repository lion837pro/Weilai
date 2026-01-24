package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static dev.nextftc.bindings.Bindings.button;

import org.firstinspires.ftc.teamcode.Robot.DriveCommands.DriveCommands;
import org.firstinspires.ftc.teamcode.Robot.Hardware.REV312010;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.ChassisConstants;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.SuperChassis;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.LED.RobotFeedback;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.ShooterCommands;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer.Spindexer;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer.SpindexerCommands;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret.Turret;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret.TurretCommands;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

/**
 * EMERGENCY TELEOP MODE - NO INTAKE
 * For competition when intake is broken/unavailable.
 * Human player feeds balls directly into the spindexer.
 *
 * Controls:
 * A = Prepare for human feed (move spindexer to intake position)
 * B = Confirm ball loaded (mark current slot as loaded)
 * X = Fixed 1600 RPM shooter
 * Y = Shoot with auto-aim (MAIN SHOOTING BUTTON)
 *
 * RB = Manual shoot at 1600 RPM (hold)
 * LB = Turret auto-align
 *
 * DpadUp = Reverse shooter
 * DpadDown = Index spindexer forward
 * DpadLeft = Reset color sort tag
 * Options = Reset heading
 *
 * RT = Manual shooter power
 * LT = Manual spindexer power
 *
 * Left stick = Drive (field-oriented)
 * Right stick = Turn
 *
 * WORKFLOW:
 * 1. Driver positions robot near human player station
 * 2. Press A to prepare spindexer for feeding
 * 3. Human player inserts ball into spindexer slot
 * 4. Press B to confirm ball is loaded
 * 5. Repeat steps 2-4 until spindexer is full (3 balls max)
 * 6. Position robot and press Y to shoot with auto-aim
 */
@TeleOp(name = "NO INTAKE EMERGENCY", group = "Emergency")
public class TeleopModeNoIntakeEmergency extends NextFTCOpMode {

    // Subsystems (NO INTAKE!)
    private final SuperChassis chassis = SuperChassis.INSTANCE;
    private final Shooter shooter = Shooter.INSTANCE;
    private final Spindexer spindexer = Spindexer.INSTANCE;
    private final Turret turret = Turret.INSTANCE;
    private REV312010 led;
    private RobotFeedback feedback;

    // Buttons
    private Button a, b, x, y;
    private Button right_bumper, left_bumper;
    private Button dpad_up, dpad_down, dpad_left;
    private Button options;

    // Constructor - Note: NO intake component!
    public TeleopModeNoIntakeEmergency() {
        addComponents(new PedroComponent(ChassisConstants::buildPedroPathing));
        addComponents(chassis.asCOMPONENT());
        addComponents(shooter.asCOMPONENT());
        addComponents(spindexer.asCOMPONENT());
        addComponents(turret.asCOMPONENT());
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
        dpad_left.whenBecomesTrue(new InstantCommand("Reset Color Sort Tag", chassis::resetColorSortTag));

        // HUMAN PLAYER FEEDING CONTROLS
        // A = Move spindexer to intake position (prepare for human to insert ball)
        a.whenBecomesTrue(SpindexerCommands.prepareForHumanFeed(spindexer));

        // B = Confirm ball loaded (mark current slot as loaded after human inserts ball)
        b.whenBecomesTrue(new InstantCommand("Confirm Ball Loaded", () -> {
            spindexer.markCurrentIntakeSlotLoaded();
            // Give feedback - rumble to confirm
            if (feedback != null) {
                feedback.rumbleDriver(0.5, 100);
            }
        }));

        // SHOOTER CONTROLS
        // X = Fixed 1600 RPM shooter (for spin-up)
        x.whenTrue(ShooterCommands.runShooterPID(shooter, 1600, feedback));
        dpad_up.whenTrue(ShooterCommands.runShooterPID(shooter, -600));

        // SHOOTING SEQUENCES (NO INTAKE!)
        // Y = Shoot with auto-aim (MAIN SHOOTING BUTTON)
        y.whenTrue(ShooterCommands.teleopShootNoIntakeAutoAim(shooter, spindexer, chassis, feedback));

        // RB = Manual shoot at fixed 1600 RPM (hold)
        right_bumper.whenTrue(ShooterCommands.teleopShootNoIntake(shooter, spindexer, 1600, feedback));

        // Turret auto-align (LB)
        left_bumper.whenTrue(TurretCommands.autoAlign(turret, chassis));

        // Spindexer manual controls
        dpad_down.whenBecomesTrue(SpindexerCommands.indexForward(spindexer));

        // Default commands (field-oriented drive)
        chassis.setDefaultCommand(DriveCommands.runWithJoysticks(chassis,
                () -> -gamepad1.left_stick_y, () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x, false));
        shooter.setDefaultCommand(ShooterCommands.runManualShooter(shooter,
                () -> gamepad1.right_trigger));
        spindexer.setDefaultCommand(SpindexerCommands.manualSpin(spindexer,
                () -> gamepad1.left_trigger));

        feedback.setReady();
    }

    @Override
    public void onWaitForStart() {
        // Auto-home spindexer during init
        telemetry.addData("=== NO INTAKE EMERGENCY MODE ===", "");
        telemetry.addData("Spindexer", "Starting homing...");
        telemetry.update();

        spindexer.startHoming();

        // Wait for limit switch with timeout
        long startTime = System.currentTimeMillis();
        long timeout = 5000;

        while (!spindexer.isAtHome() && (System.currentTimeMillis() - startTime) < timeout) {
            telemetry.addData("Homing", "Time: %dms", System.currentTimeMillis() - startTime);
            telemetry.update();

            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                break;
            }
        }

        if (spindexer.isAtHome()) {
            spindexer.finishHoming();
            telemetry.addData("Spindexer", "Homed successfully");
        } else {
            spindexer.stop();
            telemetry.addData("Spindexer", "Homing timeout - check limit switch");
        }

        // Show controls
        telemetry.addData("", "");
        telemetry.addData("=== CONTROLS ===", "");
        telemetry.addData("A", "Prepare for human feed");
        telemetry.addData("B", "Confirm ball loaded");
        telemetry.addData("Y", "Shoot with auto-aim");
        telemetry.addData("RB", "Manual shoot 1600 RPM");
        telemetry.update();

        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            // Ignore
        }
    }

    @Override
    public void onStartButtonPressed() {}

    @Override
    public void onUpdate() {
        BindingManager.update();
        if (feedback != null) feedback.update();

        // Telemetry
        telemetry.addData("=== NO INTAKE MODE ===", "");
        telemetry.addData("Balls Loaded", "%d / 3", spindexer.getBallCount());

        // Show slot status
        String slot0 = spindexer.hasBall(0) ? "FULL" : "empty";
        String slot1 = spindexer.hasBall(1) ? "FULL" : "empty";
        String slot2 = spindexer.hasBall(2) ? "FULL" : "empty";
        telemetry.addData("Slots", "[%s | %s | %s]", slot0, slot1, slot2);

        telemetry.addData("Spindexer Pos", "%s", spindexer.isAtIntakePosition() ? "INTAKE" : "SHOOTER");
        telemetry.addData("Feeder", spindexer.isFeederUp() ? "UP" : "DOWN");
        telemetry.addData("", "");
        telemetry.addData("A=Prepare | B=Confirm", "Y=Shoot");
        telemetry.update();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
        if (feedback != null) feedback.setIdle();
    }
}
