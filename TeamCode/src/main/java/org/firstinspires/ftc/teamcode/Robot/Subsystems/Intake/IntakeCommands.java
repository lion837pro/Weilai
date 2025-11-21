package org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;

public class IntakeCommands {

    public static Command runIntake(Intake intake, double speed){

        return  new LambdaCommand().
                named("runIntake").
                requires(intake).
                setStart(()-> {}).
                setUpdate(()-> { intake.MoveIn(speed);}).
                setStop(interrupted-> { intake.MoveIn(0);

                }).
                setIsDone(()-> true).
                setInterruptible(true);
    }
    public static Command stopIntake(Intake intake) {
        return new LambdaCommand()
                .named("stopIntake")
                .requires(intake) // This requirement forces the 'run' command to cancel!
                .setStart(() -> intake.MoveIn(0))
                .setUpdate(() -> intake.MoveIn(0))
                .setIsDone(() -> true) // Finishes instantly
                .setInterruptible(true);
    }
}






