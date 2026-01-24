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
 * EMERGENCY TELEOP MODE - NO INTAKE, NO LIMELIGHT
 * For competition when intake and/or Limelight are broken/unavailable.
 * Human player feeds balls directly into the spindexer.
 * Turret is manually controlled (no auto-aim).
 *
 * BEFORE STARTING:
 * 1. Position turret pointing FORWARD (center)
 * 2. Position spindexer with slot at limit switch
 *
 * Controls:
 * A = Prepare for human feed (move spindexer to intake position)
 * B = Confirm ball loaded (mark current slot as loaded)
 * X = Fixed 1600 RPM shooter (spin-up only)
 * Y = Shoot at 1600 RPM (MAIN SHOOTING BUTTON)
 *
 * RB = Shoot at 1800 RPM (higher power)
 * LB = Re-zero turret (if position drifts)
 *
 * DpadUp = Reverse shooter
 * DpadDown = Index spindexer forward
 * DpadLeft = Turret go to center
 * DpadRight = Turret go to right 45°
 * Options = Reset heading
 *
 * RT = Manual shooter power
 * LT = Manual spindexer power
 *
 * Left stick = Drive (field-oriented)
 * Right stick X = TURRET MANUAL CONTROL
 *
 * WORKFLOW:
 * 1. Driver positions robot near human player station
 * 2. Press A to prepare spindexer for feeding
 * 3. Human player inserts ball into spindexer slot
 * 4. Press B to confirm ball is loaded
 * 5. Repeat steps 2-4 until spindexer is full (3 balls max)
 * 6. Use right stick to aim turret manually
 * 7. Press Y to shoot at 1600 RPM
 */
@TeleOp(name = "NO INTAKE EMERGENCY", group = "Emergency")
public class TeleopModeNoIntakeEmergency extends NextFTCOpMode {

    // Subsystems (NO INTAKE, NO LIMELIGHT AUTO-AIM!)
    private final SuperChassis chassis = SuperChassis.INSTANCE;
    private final Shooter shooter = Shooter.INSTANCE;
    private final Spindexer spindexer = Spindexer.INSTANCE;
    private final Turret turret = Turret.INSTANCE;
    private REV312010 led;
    private RobotFeedback feedback;

    // Buttons
    private Button a, b, x, y;
    private Button right_bumper, left_bumper;
    private Button dpad_up, dpad_down, dpad_left, dpad_right;
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
        this.dpad_right = button(() -> gamepad1.dpad_right);

        // System controls
        options.whenBecomesTrue(DriveCommands.resetHeading(chassis));

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
        // X = Fixed 1600 RPM shooter (for spin-up only)
        x.whenTrue(ShooterCommands.runShooterPID(shooter, 1600, feedback));
        dpad_up.whenTrue(ShooterCommands.runShooterPID(shooter, -600));

        // SHOOTING SEQUENCES (NO INTAKE, FIXED RPM - NO AUTO-AIM!)
        // Y = Shoot at 1600 RPM (MAIN SHOOTING BUTTON)
        y.whenTrue(ShooterCommands.teleopShootNoIntake(shooter, spindexer, 1600, feedback));

        // RB = Shoot at higher 1800 RPM for longer distance
        right_bumper.whenTrue(ShooterCommands.teleopShootNoIntake(shooter, spindexer, 1800, feedback));

        // LB = Re-zero turret (if position drifts during match)
        left_bumper.whenBecomesTrue(TurretCommands.zero(turret));

        // TURRET PRESET POSITIONS
        dpad_left.whenBecomesTrue(TurretCommands.goToCenter(turret));
        dpad_right.whenBecomesTrue(TurretCommands.goToRight45(turret));

        // Spindexer manual controls
        dpad_down.whenBecomesTrue(SpindexerCommands.indexForward(spindexer));

        // Default commands
        // Drive with left stick, but RIGHT STICK X controls turret (not turning!)
        chassis.setDefaultCommand(DriveCommands.runWithJoysticks(chassis,
                () -> -gamepad1.left_stick_y, () -> -gamepad1.left_stick_x,
                () -> 0, false));  // No chassis turning on right stick

        // TURRET: Manual control with right stick X
        turret.setDefaultCommand(TurretCommands.manualControl(turret,
                () -> -gamepad1.right_stick_x));

        shooter.setDefaultCommand(ShooterCommands.runManualShooter(shooter,
                () -> gamepad1.right_trigger));
        spindexer.setDefaultCommand(SpindexerCommands.manualSpin(spindexer,
                () -> gamepad1.left_trigger));

        feedback.setReady();
    }

    @Override
    public void onWaitForStart() {
        // CRITICAL: Zero turret first
        telemetry.addData("=== EMERGENCY MODE ===", "NO INTAKE / NO LIMELIGHT");
        telemetry.addData("", "");
        telemetry.addData("!! IMPORTANT !!", "Position TURRET facing FORWARD");
        telemetry.addData("", "Turret will be ZEROED at current position");
        telemetry.update();

        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            // Ignore
        }

        // Zero the turret
        turret.zero();
        telemetry.addData("Turret", "ZEROED at current position");
        telemetry.update();

        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            // Ignore
        }

        // Home spindexer
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
            telemetry.addData("Spindexer", "Homing timeout");
        }

        // Show controls
        telemetry.addData("", "");
        telemetry.addData("=== CONTROLS ===", "");
        telemetry.addData("A", "Prepare for feed");
        telemetry.addData("B", "Confirm ball");
        telemetry.addData("Y", "Shoot 1600 RPM");
        telemetry.addData("RB", "Shoot 1800 RPM");
        telemetry.addData("Right Stick X", "AIM TURRET");
        telemetry.addData("LB", "Re-zero turret");
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
        telemetry.addData("=== EMERGENCY MODE ===", "");

        // Turret status
        if (!turret.isZeroed()) {
            telemetry.addData("!! TURRET !!", "NOT ZEROED - Press LB");
        }
        telemetry.addData("Turret", "%.1f° (limit ±90°)", turret.getCurrentAngle());

        // Ball status
        telemetry.addData("Balls", "%d / 3", spindexer.getBallCount());
        String slot0 = spindexer.hasBall(0) ? "FULL" : "empty";
        String slot1 = spindexer.hasBall(1) ? "FULL" : "empty";
        String slot2 = spindexer.hasBall(2) ? "FULL" : "empty";
        telemetry.addData("Slots", "[%s | %s | %s]", slot0, slot1, slot2);

        telemetry.addData("Spindexer", "%s", spindexer.isAtIntakePosition() ? "INTAKE" : "SHOOTER");
        telemetry.addData("", "");
        telemetry.addData("A=Feed B=Confirm", "Y=Shoot");
        telemetry.addData("Right Stick X", "Aim turret");
        telemetry.update();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
        if (feedback != null) feedback.setIdle();
    }
}
