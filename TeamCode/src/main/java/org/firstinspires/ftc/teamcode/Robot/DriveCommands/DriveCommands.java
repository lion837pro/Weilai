package org.firstinspires.ftc.teamcode.Robot.DriveCommands;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Lib.STZLite.Geometry.Rotation;
import org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Utils.Units;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.SuperChassis;

import java.util.function.DoubleSupplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.TurnBy;
import dev.nextftc.extensions.pedro.TurnTo;

public class DriveCommands {

    public static Command runWithJoysticks(SuperChassis chasis, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn, boolean robotCentric){
        return new LambdaCommand().
                named("RunWithJoysticks").
                requires(chasis).
                setStart(follower()::startTeleOpDrive).
                setUpdate(
                        ()-> {

                            double fw = forward.getAsDouble();
                            double st = strafe.getAsDouble();
                            double tr = turn.getAsDouble();

                            follower().setTeleOpDrive(fw, st, tr, robotCentric);
                        }
                ).
                setStop(interrupted
                                -> {
                            chasis.stop();
                        }
                ).
                setIsDone(()-> false).
                setInterruptible(true);
    }

    public static Command resetHeading(SuperChassis chassis){
        return new InstantCommand("Reset Heading", chassis::resetHeading);
    }

    public static Command toPath(SuperChassis chassis, Path path, boolean hold, double maxPower){
        Command follower = new FollowPath(path, hold, maxPower);
        follower.named("Follow Command").requires(chassis);

        return  follower;

    }

    public static Command toPath(SuperChassis chassis, Path path, boolean hold){
        return toPath(chassis, path, hold, 1);
    }

    public static Command toPath(SuperChassis chassis, Path path){
        return toPath(chassis, path, false);
    }

    public static Command Turn(SuperChassis chassis, double angleDeg){
        double radTo = Units.degreesToRadians(angleDeg);
        Command turn = new TurnTo(Angle.fromRad(radTo));
        turn.named("Turn Command").requires(chassis);
        return turn;
    }

    public static Command Turn(SuperChassis chassis, Angle angleRads) {
        Command turn = new TurnTo(angleRads);
        turn.named("Turn Command").requires(chassis);
        return turn;
    }

    public static Command Turn(SuperChassis chassis, Rotation rotRads){
        return Turn(chassis, rotRads.toAngle());
    }

    public static Command TurnBy(SuperChassis chassis, double angleDeg){
        double radTo = Units.degreesToRadians(angleDeg);
        Command turn = new TurnBy(Angle.fromRad(radTo));
        turn.named("TurnBy Command").requires(chassis);
        return turn;
    }

    public static Command TurnBy(SuperChassis chassis, Angle angleRads) {
        Command turn = new TurnBy(angleRads);
        turn.named("TurnBy Command").requires(chassis);
        return turn;
    }

    public static Command TurnBy(SuperChassis chassis, Rotation rotRads){
        return TurnBy(chassis, rotRads.toAngle());
    }

}
