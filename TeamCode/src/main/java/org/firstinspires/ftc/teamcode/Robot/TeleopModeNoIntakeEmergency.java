package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import static dev.nextftc.bindings.Bindings.button;

import org.firstinspires.ftc.teamcode.Robot.DriveCommands.DriveCommands;
import org.firstinspires.ftc.teamcode.Robot.Hardware.REV312010;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.ChassisConstants;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.SuperChassis;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.LED.RobotFeedback;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.ShooterEmergency;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer.Spindexer;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer.SpindexerConstants;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

/**
 * EMERGENCY TELEOP MODE - NO INTAKE, NO TURRET, REV MOTOR SHOOTER
 * For competition when intake/Limelight/turret are broken.
 * Human player feeds balls directly into the spindexer.
 * Shooter uses simple power control.
 *
 * TWO DRIVER CONFIGURATION:
 * - DRIVER 1 (gamepad1): Full chassis control
 * - DRIVER 2 (gamepad2): Shooter and spindexer
 *
 * === DRIVER 1 CONTROLS (gamepad1) ===
 * Left stick = Drive (field-oriented)
 * Right stick X = Turn chassis
 * Options = Reset heading
 *
 * === DRIVER 2 CONTROLS (gamepad2) ===
 * A = Prepare for human feed (move spindexer to intake position)
 * B = Confirm ball loaded (mark current slot as loaded)
 * X = Medium power shooter (spin-up only)
 * Y = SHOOT (cycles feeder servo)
 *
 * RB = Shoot at high power
 * LB = Feeder servo down (reset)
 *
 * DpadUp = Reverse shooter (slow)
 * DpadDown = Index spindexer forward
 *
 * RT = Manual shooter power
 * LT = Manual spindexer power
 */
@TeleOp(name = "NO INTAKE EMERGENCY", group = "Emergency")
public class TeleopModeNoIntakeEmergency extends NextFTCOpMode {

    // Subsystems (NO INTAKE, NO TURRET!)
    private final SuperChassis chassis = SuperChassis.INSTANCE;
    private final ShooterEmergency shooter = ShooterEmergency.INSTANCE;
    private final Spindexer spindexer = Spindexer.INSTANCE;
    private REV312010 led;
    private RobotFeedback feedback;

    // Feeder servo cycling
    private ElapsedTime feederTimer = new ElapsedTime();
    private boolean feederIsUp = false;
    private static final long FEEDER_CYCLE_TIME_MS = 250;

    // Buttons
    private Button a, b, x, y;
    private Button right_bumper, left_bumper;
    private Button dpad_up, dpad_down;
    private Button options;

    // Constructor
    public TeleopModeNoIntakeEmergency() {
        addComponents(new PedroComponent(ChassisConstants::buildPedroPathing));
        addComponents(chassis.asCOMPONENT());
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
        // Driver 1 (gamepad1): Only options for heading reset
        this.options = button(() -> gamepad1.options);

        // Driver 2 (gamepad2): All shooter/spindexer controls
        this.a = button(() -> gamepad2.a);
        this.b = button(() -> gamepad2.b);
        this.x = button(() -> gamepad2.x);
        this.y = button(() -> gamepad2.y);
        this.right_bumper = button(() -> gamepad2.right_bumper);
        this.left_bumper = button(() -> gamepad2.left_bumper);
        this.dpad_up = button(() -> gamepad2.dpad_up);
        this.dpad_down = button(() -> gamepad2.dpad_down);

        // System controls
        options.whenBecomesTrue(DriveCommands.resetHeading(chassis));

        // HUMAN PLAYER FEEDING CONTROLS
        // A = Move spindexer to intake position
        a.whenBecomesTrue(new InstantCommand("PrepareForFeed", () -> {
            spindexer.moveToIntakePosition();
            if (feedback != null) feedback.rumbleGamepad2(0.3, 100);
        }));

        // B = Confirm ball loaded
        b.whenBecomesTrue(new InstantCommand("ConfirmBall", () -> {
            spindexer.markCurrentIntakeSlotLoaded();
            if (feedback != null) feedback.rumbleGamepad2(0.5, 100);
        }));

        // SHOOTER CONTROLS
        // X = Spin up shooter (hold)
        x.whenTrue(createShooterCommand(0.5));

        // DpadUp = Reverse shooter
        dpad_up.whenTrue(createShooterCommand(-0.2));

        // Y = SHOOT - cycles feeder servo while held
        y.whenTrue(createShootCommand(0.5));

        // RB = Shoot at higher power
        right_bumper.whenTrue(createShootCommand(0.6));

        // LB = Reset feeder servo down
        left_bumper.whenBecomesTrue(new InstantCommand("FeederDown", () -> {
            spindexer.feederDown();
            feederIsUp = false;
        }));

        // Spindexer manual controls
        dpad_down.whenBecomesTrue(new InstantCommand("IndexForward", () -> {
            spindexer.indexToNextSlot();
        }));

        // Default commands
        // DRIVER 1: Full chassis control
        chassis.setDefaultCommand(DriveCommands.runWithJoysticks(chassis,
                () -> -gamepad1.left_stick_y, () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x, false));

        // DRIVER 2: Manual shooter with right trigger
        shooter.setDefaultCommand(createManualShooterCommand());

        // DRIVER 2: Manual spindexer with left trigger
        spindexer.setDefaultCommand(createManualSpindexerCommand());

        feedback.setReady();
    }

    /**
     * Create a shooter spin command (no feeding)
     */
    private LambdaCommand createShooterCommand(double power) {
        return new LambdaCommand()
                .named("SpinShooter")
                .requires(shooter)
                .setStart(() -> shooter.setPower(power))
                .setUpdate(() -> shooter.setPower(power))
                .setStop(interrupted -> shooter.stop())
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    /**
     * Create a shoot command that spins shooter AND cycles feeder servo
     */
    private LambdaCommand createShootCommand(double power) {
        return new LambdaCommand()
                .named("Shoot")
                .requires(shooter, spindexer)
                .setStart(() -> {
                    shooter.setPower(power);
                    feederTimer.reset();
                    feederIsUp = false;
                    spindexer.feederDown();
                })
                .setUpdate(() -> {
                    shooter.setPower(power);

                    // Cycle feeder: up -> wait -> down -> wait -> repeat
                    long elapsed = (long) feederTimer.milliseconds();

                    if (!feederIsUp && elapsed >= FEEDER_CYCLE_TIME_MS) {
                        // Time to push up
                        spindexer.feederUp();
                        feederIsUp = true;
                        feederTimer.reset();
                    } else if (feederIsUp && elapsed >= FEEDER_CYCLE_TIME_MS) {
                        // Time to go back down
                        spindexer.feederDown();
                        feederIsUp = false;
                        feederTimer.reset();
                        // Mark ball as shot
                        if (spindexer.hasBall(spindexer.getShooterSlot())) {
                            spindexer.markBallShot(spindexer.getShooterSlot());
                        }
                    }
                })
                .setStop(interrupted -> {
                    shooter.stop();
                    spindexer.feederDown();
                    feederIsUp = false;
                })
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    /**
     * Manual shooter control with trigger
     */
    private LambdaCommand createManualShooterCommand() {
        return new LambdaCommand()
                .named("ManualShooter")
                .requires(shooter)
                .setStart(() -> {})
                .setUpdate(() -> {
                    double power = gamepad2.right_trigger;
                    if (power > 0.05) {
                        shooter.setPower(power * 0.7); // Scale to 70% max
                    } else {
                        shooter.stop();
                    }
                })
                .setStop(interrupted -> shooter.stop())
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    /**
     * Manual spindexer control with trigger
     */
    private LambdaCommand createManualSpindexerCommand() {
        return new LambdaCommand()
                .named("ManualSpindexer")
                .requires(spindexer)
                .setStart(() -> {})
                .setUpdate(() -> {
                    double power = gamepad2.left_trigger;
                    if (power > 0.05) {
                        spindexer.spin(power * 0.5); // Scale to 50%
                    } else {
                        spindexer.stop();
                    }
                })
                .setStop(interrupted -> spindexer.stop())
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    @Override
    public void onWaitForStart() {
        telemetry.addData("=== EMERGENCY MODE ===", "NO TURRET / NO INTAKE");
        telemetry.addData("", "");
        telemetry.addData("Shooter", "REV Motor");
        telemetry.addData("Feeder", "Servo cycles on Y/RB");
        telemetry.update();

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
            telemetry.addData("Spindexer", "Homing timeout - using manual");
        }

        // Set feeder down initially
        spindexer.feederDown();

        // Show controls
        telemetry.addData("", "");
        telemetry.addData("=== 2 DRIVER MODE ===", "");
        telemetry.addData("DRIVER 1", "Full chassis (drive+turn)");
        telemetry.addData("DRIVER 2", "Shooter/Spindexer/Feeder");
        telemetry.addData("", "");
        telemetry.addData("D2: A", "Prepare for feed");
        telemetry.addData("D2: B", "Confirm ball");
        telemetry.addData("D2: Y", "SHOOT (cycles feeder)");
        telemetry.addData("D2: RB", "Shoot high power");
        telemetry.addData("D2: LB", "Reset feeder down");
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
        telemetry.addData("=== EMERGENCY MODE ===", "NO TURRET");

        // Ball status
        telemetry.addData("Balls", "%d / 3", spindexer.getBallCount());
        String slot0 = spindexer.hasBall(0) ? "FULL" : "empty";
        String slot1 = spindexer.hasBall(1) ? "FULL" : "empty";
        String slot2 = spindexer.hasBall(2) ? "FULL" : "empty";
        telemetry.addData("Slots", "[%s | %s | %s]", slot0, slot1, slot2);

        telemetry.addData("Spindexer", "%s", spindexer.isAtIntakePosition() ? "INTAKE" : "SHOOTER");
        telemetry.addData("Shooter RPM", "%.0f", shooter.getCurrentRPM());
        telemetry.addData("Feeder", feederIsUp ? "UP" : "DOWN");
        telemetry.addData("", "");
        telemetry.addData("D1: Drive+Turn", "D2: Shoot/Feed");
        telemetry.addData("D2: Y=Shoot", "LB=Reset feeder");
        telemetry.update();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
        spindexer.feederDown();
        if (feedback != null) feedback.setIdle();
    }
}
