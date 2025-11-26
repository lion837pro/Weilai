package org.firstinspires.ftc.teamcode.Robot.Emergencia;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.ChassisConstants;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.SuperChassis;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

@Autonomous(name = "Simple Auto 2 Sec")
public class SimpleAutonomous extends NextFTCOpMode {

    private final SuperChassis chassis = SuperChassis.INSTANCE;

    public SimpleAutonomous() {
        // ✅ FIXED: Correctly register PedroComponent so it doesn't crash
        addComponents(new PedroComponent(ChassisConstants::buildPedroPathing));
        addComponents(chassis.asCOMPONENT());
    }

    @Override
    public void onInit() {
        // Safely set start pose
        try {
            if (follower() != null) {
                follower().setStartingPose(new com.pedropathing.geometry.Pose(0,0,0));
            }
        } catch (Exception e) {}
    }

    @Override
    public void onStartButtonPressed() {
        // Drive Forward Command
        Command driveForward = new LambdaCommand()
                .named("DriveForwardTime")
                .requires(chassis)
                .setStart(() -> {
                    if (follower() != null) follower().startTeleOpDrive();
                })
                .setUpdate(() -> {
                    // 30% Power Forward
                    if (follower() != null) {
                        follower().setTeleOpDrive(0.3, 0, 0, true); 
                    }
                })
                .setStop((interrupted) -> {
                    // ✅ STOP: Cut power when command ends
                    if (follower() != null) {
                        follower().setTeleOpDrive(0, 0, 0, true);
                    }
                })
                .setIsDone(() -> false) // Run forever...
                .raceWith(new Delay(2.0)); // ...until 2 seconds pass.

        // Schedule it
        driveForward.invoke();
    }
}