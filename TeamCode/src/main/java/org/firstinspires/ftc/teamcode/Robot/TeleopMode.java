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

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "NextFTC Teleop")
public class TeleopMode extends NextFTCOpMode {

    private final SuperChassis chassis = SuperChassis.INSTANCE;
    private final Intake intake = Intake.INSTANCE;
    private final Shooter shooter = Shooter.INSTANCE;


    private  Button a;
    private  Button b;
    private Button right_bumper;
    private Button left_bumper;
    private Button x;
    private  Button options;

    public TeleopMode() {
        addComponents(new PedroComponent(ChassisConstants::buildPedroPathing));
        addComponents(chassis.asCOMPONENT());
        addComponents(intake.asCOMPONENT());
        addComponents(shooter.asCOMPONENT());




    }

    @Override
    public void onInit() {

        this.a = button(() -> gamepad1.a);
        this.b = button(() -> gamepad1.b);
        //this.x = button(() -> gamepad1.x);
        this.right_bumper = button(() -> gamepad1.right_bumper);
        this.left_bumper = button(() -> gamepad1.left_bumper);
        this.options = button(() -> gamepad1.options);


        options.whenBecomesTrue(DriveCommands.resetHeading(chassis));


        a.whenBecomesTrue(IntakeCommands.runIntake(intake, 0.8));
        a.whenBecomesFalse(IntakeCommands.stopIntake(intake));

        b.whenBecomesTrue(IntakeCommands.runIntake(intake, -0.8));
        b.whenBecomesFalse(IntakeCommands.stopIntake(intake));

        //x.whenBecomesTrue(ShooterCommands.runShooter(shooter, 0.8));

        right_bumper.whenBecomesTrue(ShooterCommands.runShooterPID(shooter, 1800));
        right_bumper.whenBecomesFalse(ShooterCommands.stopShooter(shooter));

        left_bumper.whenBecomesTrue(DriveCommands.AlignByAprilTag(chassis));

            
        shooter.setDefaultCommand(
                ShooterCommands.runManualShooter(shooter,
                        () -> gamepad1.right_trigger));


        // 4. Drive Command (This part is correct)
        chassis.setDefaultCommand(
                DriveCommands.runWithJoysticks(chassis,
                        () -> -gamepad1.left_stick_y,
                        () -> -gamepad1.left_stick_x,
                        () -> -gamepad1.right_stick_x,
                        false));
    }
    @Override
    public void onWaitForStart() {}
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
