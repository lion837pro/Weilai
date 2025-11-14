package org.firstinspires.ftc.teamcode.Robot;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.DriveCommands.DriveCommands;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.SuperChassis;

import dev.nextftc.bindings.Button;
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "NextFTC Teleop Program 2 Java")
public class TeleopMode extends NextFTCOpMode {

    private final SuperChassis chassis = SuperChassis.INSTANCE;

    private final Button a;
    private final Button b;
    private final Button options;

    public TeleopMode(){
        addComponents(chassis.asCOMPONENT());

        this.a = button(() -> gamepad1.a);
        this.b = button(() -> gamepad1.b);
        this.options = button(() -> gamepad1.options);

    }

    @Override
    public void onInit() {

    }

    @Override
    public void onStartButtonPressed() {

        options.whenTrue(DriveCommands.resetHeading(chassis));

    }

}
