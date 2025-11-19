package org.firstinspires.ftc.teamcode.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static dev.nextftc.bindings.Bindings.button;
import org.firstinspires.ftc.teamcode.Robot.DriveCommands.DriveCommands;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.SuperChassis;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.IntakeCommands;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.Shooter;
import dev.nextftc.bindings.Button;
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "NextFTC Teleop")
public class TeleopMode extends NextFTCOpMode {

    private final SuperChassis chassis = SuperChassis.INSTANCE;
    private final Intake intake = Intake.INSTANCE;
    private final Shooter shooter = Shooter.INSTANCE;


    private final Button a;
    private final Button b;
    private final Button options;

    public TeleopMode() {
        addComponents(chassis.asCOMPONENT());
        addComponents(intake.asCOMPONENT());
        addComponents(shooter.asCOMPONENT());

        this.a = button(() -> gamepad1.a);
        this.b = button(() -> gamepad1.b);
        this.options = button(() -> gamepad1.options);

    }

    @Override
    public void onInit() {
        options.whenTrue(DriveCommands.resetHeading(chassis));
        a.whenTrue(IntakeCommands.runIntake(intake, 0.8));
        b.whenTrue(IntakeCommands.runIntake(intake, -0.8));
    }
    @Override
    public void onWaitForStart() {}
    @Override
    public void onStartButtonPressed() {}
    @Override
    public void onUpdate() {}
    @Override
    public void onStop() {}
}
