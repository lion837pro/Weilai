package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static dev.nextftc.bindings.Bindings.button;

import org.firstinspires.ftc.teamcode.Robot.DriveCommands.DriveCommands;
import org.firstinspires.ftc.teamcode.Robot.Hardware.REV312010;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.ChassisConstants;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.SuperChassis;
// import org.firstinspires.ftc.teamcode.Robot.Subsystems.Hood.Hood;  // Hood disabled - using RPM-based distance
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.IntakeCommands;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.LED.RobotFeedback;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.ShooterCommands;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer.Spindexer;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer.SpindexerCommands;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer.SpindexerConstants;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret.Turret;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret.TurretCommands;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

import java.util.ArrayList;
import java.util.List;

/**
 * 2-DRIVER TELEOP MODE
 * GP1(Driver): A=Intake | B=Reverse | X=1600RPM | Y=FixedRPM+Spindexer | RB=AutoAimRPM(MAIN)
 *              LB=TurretAutoAlign | RT=ManualShooter | Options=ResetHeading | DpadLeft=ResetColorTag
 *
 * GP2(Operator): Advanced spindexer control
 * GP2 PROGRAMMING: Hold LB + press A/B/X to build sequence → Release LB → Press Y to execute
 * GP2 SLOTS: A=Slot0 | B=Slot1 | X=Slot2 | Y=ExecuteSequence | RB=ResetSequence
 * GP2 MANUAL: LT=SpinBack | RT=SpinFwd | DpadUp=Home | DpadDown=IndexFwd | DpadLeft=IndexBack | DpadRight=PrepIntake
 */
@TeleOp(name = "2 Driver Mode", group = "Competition")
public class TeleopMode2Driver extends NextFTCOpMode {

    // Subsystems
    private final SuperChassis chassis = SuperChassis.INSTANCE;
    private final Intake intake = Intake.INSTANCE;
    private final Shooter shooter = Shooter.INSTANCE;
    private final Spindexer spindexer = Spindexer.INSTANCE;
    // private final Hood hood = Hood.INSTANCE;  // Hood disabled - using RPM-based distance
    private final Turret turret = Turret.INSTANCE;
    private REV312010 led;
    private RobotFeedback feedback;

    // GP1 buttons
    private Button gp1_a, gp1_b, gp1_x, gp1_y;
    private Button gp1_right_bumper, gp1_left_bumper;
    private Button gp1_options, gp1_dpad_up, gp1_dpad_down, gp1_dpad_left;

    // GP2 buttons
    private Button gp2_a, gp2_b, gp2_x, gp2_y;
    private Button gp2_right_bumper, gp2_left_bumper;
    private Button gp2_dpad_up, gp2_dpad_down, gp2_dpad_left, gp2_dpad_right;

    // Custom sequence state
    private final List<Integer> customSequence = new ArrayList<>();
    private boolean programmingMode = false;

    // Constructor
    public TeleopMode2Driver() {
        addComponents(new PedroComponent(ChassisConstants::buildPedroPathing));
        addComponents(chassis.asCOMPONENT());
        addComponents(intake.asCOMPONENT());
        addComponents(shooter.asCOMPONENT());
        addComponents(spindexer.asCOMPONENT());
        // addComponents(hood.asCOMPONENT());  // Hood disabled - using RPM-based distance
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

        // Initialize GP1 buttons
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

        // Initialize GP2 buttons
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

        // ========== GP1 DRIVER CONTROLS ==========

        // System controls (instant commands)
        gp1_options.whenBecomesTrue(DriveCommands.resetHeading(chassis));
        gp1_dpad_left.whenBecomesTrue(new InstantCommand("Reset Color Sort Tag", chassis::resetColorSortTag));

        // Intake controls - use whenTrue for proper "run while held" behavior
        gp1_a.whenTrue(IntakeCommands.runIntakeWithSpindexer(spindexer, intake, 0.7, feedback));
        gp1_b.whenTrue(IntakeCommands.runIntake(intake, -0.7));

        // Shooter controls - whenTrue auto-cancels on release
        gp1_x.whenTrue(ShooterCommands.runShooterPID(shooter, 1600, feedback));
        gp1_dpad_up.whenTrue(ShooterCommands.runShooterPID(shooter, -600));

        // Full shooting sequences (PRIMARY COMPETITION CONTROLS)
        // RB: RPM-based auto-aim with color sorting (MAIN SHOOTING BUTTON)
        gp1_right_bumper.whenTrue(ShooterCommands.teleopShootColorSortedAutoAim(
                shooter, spindexer, intake, chassis, feedback));

        // Y: Fixed RPM shooting without hood auto-aim
        gp1_y.whenTrue(ShooterCommands.teleopShootFixedRPM(shooter, spindexer, intake, 1600, feedback));

        // Turret auto-align (LB) - uses Limelight to aim turret instead of chassis
        gp1_left_bumper.whenTrue(TurretCommands.autoAlign(turret, chassis));

        // Spindexer manual controls
        gp1_dpad_down.whenBecomesTrue(SpindexerCommands.indexForward(spindexer));

        // GP1 default commands
        chassis.setDefaultCommand(DriveCommands.runWithJoysticks(chassis,
                () -> -gamepad1.left_stick_y, () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x, false));
        shooter.setDefaultCommand(ShooterCommands.runManualShooter(shooter,
                () -> gamepad1.right_trigger));

        // ========== GP2 OPERATOR CONTROLS ==========

        // GP2 programming mode
        gp2_left_bumper.whenBecomesTrue(new InstantCommand("Enter Programming Mode", () -> {
            programmingMode = true;
            gamepad2.rumble(200);
        }));
        gp2_left_bumper.whenBecomesFalse(new InstantCommand("Exit Programming Mode", () -> {
            programmingMode = false;
            if (!customSequence.isEmpty()) gamepad2.rumble(100);
        }));

        // GP2 slot selection/programming
        gp2_a.whenBecomesTrue(new InstantCommand("Slot 0 Action", () -> {
            if (programmingMode) {
                customSequence.add(0);
                gamepad2.rumble(50);
            } else {
                spindexer.goToPosition(SpindexerConstants.getShooterPosition(0));
            }
        }));
        gp2_b.whenBecomesTrue(new InstantCommand("Slot 1 Action", () -> {
            if (programmingMode) {
                customSequence.add(1);
                gamepad2.rumble(50);
            } else {
                spindexer.goToPosition(SpindexerConstants.getShooterPosition(1));
            }
        }));
        gp2_x.whenBecomesTrue(new InstantCommand("Slot 2 Action", () -> {
            if (programmingMode) {
                customSequence.add(2);
                gamepad2.rumble(50);
            } else {
                spindexer.goToPosition(SpindexerConstants.getShooterPosition(2));
            }
        }));

        // GP2 sequence execution - whenTrue auto-cancels on release
        gp2_y.whenTrue(createCustomSequenceCommand());

        gp2_right_bumper.whenBecomesTrue(new InstantCommand("Reset Custom Sequence", () -> {
            customSequence.clear();
            gamepad2.rumble(300);
        }));

        // GP2 manual spindexer controls
        gp2_dpad_up.whenBecomesTrue(SpindexerCommands.homeSpindexer(spindexer));
        gp2_dpad_down.whenBecomesTrue(SpindexerCommands.indexForward(spindexer));
        gp2_dpad_left.whenBecomesTrue(SpindexerCommands.indexBackward(spindexer));
        gp2_dpad_right.whenBecomesTrue(SpindexerCommands.prepareForIntake(spindexer));

        // GP2 default command
        spindexer.setDefaultCommand(SpindexerCommands.manualSpin(spindexer,
                () -> gamepad2.right_trigger - gamepad2.left_trigger));

        feedback.setReady();
    }

    // Helper: Create custom sequence command
    private dev.nextftc.core.commands.Command createCustomSequenceCommand() {
        return ShooterCommands.teleopShootCustomSequence(
                shooter, spindexer, intake, 1800, customSequence, feedback);
    }

    // Helper: Format sequence for telemetry
    private String formatSequence() {
        if (customSequence.isEmpty()) return "[Empty - Hold LB + A/B/X]";
        StringBuilder sb = new StringBuilder("[");
        for (int i = 0; i < customSequence.size(); i++) {
            sb.append(customSequence.get(i));
            if (i < customSequence.size() - 1) sb.append(" → ");
        }
        sb.append("]");
        return sb.toString();
    }

    // Lifecycle methods
    @Override
    public void onWaitForStart() {
        // Auto-home spindexer during init
        telemetry.addData("Spindexer", "Starting homing...");
        telemetry.update();

        spindexer.startHoming();

        long startTime = System.currentTimeMillis();
        long timeout = 5000;

        while (!spindexer.isAtHome() && (System.currentTimeMillis() - startTime) < timeout) {
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
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {}

    @Override
    public void onUpdate() {
        BindingManager.update();
        if (feedback != null) feedback.update();

        // Telemetry
        telemetry.addData("Color Sort", chassis.getColorSortModeString());
        telemetry.addData("Programming", programmingMode ? "ACTIVE" : "Inactive");
        telemetry.addData("Sequence", formatSequence());
        // telemetry.addData("Hood Angle", "%.1f deg", hood.getAngleDegrees());  // Hood disabled
        telemetry.addData("Turret Aligned", turret.isAligned() ? "YES" : "NO");
        telemetry.update();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
        if (feedback != null) feedback.setIdle();
    }
}
