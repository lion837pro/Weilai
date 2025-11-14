package org.firstinspires.ftc.teamcode.Intake;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;

public class intakeCommands {

    public static Command runWithButton (Intake intake, double speed){
        return new LambdaCommand ().
                named ("runWithButton").
                requires(intake).
                setStart(null).
                setUpdate(
                        ()-> { intake.MoveIn(speed);
                        }
                ).setStop(()->{ intake.MoveIn(0);

                }).setIsDone(()->{ intake.MoveIn(0);
                    return null;
                });
    }
    public static Command runWithButton(Intake intake, double speed){
        return  new LambdaCommand().
                named("runWithButton").
                requires(null).
                setStart(()-> {

                }).
                setUpdate(()-> {

                }).
                setStop(interrupted-> {

                }).
                setIsDone(()-> false).setInterruptible(true).
                setInterruptible(false);
}
}





