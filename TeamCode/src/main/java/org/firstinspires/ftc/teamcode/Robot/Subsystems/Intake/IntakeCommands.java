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
                setIsDone(()-> false).
                setInterruptible(true);
    }
}





