package org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.Intake;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;

public class ShooterCommands {

    public static Command runShooter(Shooter shooter, double speed) {

        return new LambdaCommand().
                named("runWithButton").
                requires(shooter).
                setStart(() -> {}).
                setUpdate(() -> { shooter.MoveSh(speed);}).
                setStop(interrupted -> { shooter.MoveSh(0);

                }).
                setIsDone(() -> false).
                setInterruptible(true);
    }
}
