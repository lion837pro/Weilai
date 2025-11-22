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
import dev.nextftc.ftc.ActiveOpMode;

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
    public static Command AlignByAprilTag(SuperChassis chassis){

        return new LambdaCommand().
                named("AlignByAprilTag").
                requires(chassis).
                setStart(()-> {

                    if(chassis.isLLConnected() && chassis.getLIMELIGHT().getLatestResult().isValid())
                    {
                        double heading = chassis.getFOLLOWER().getPose().getHeading();
                        double tx = Math.toRadians(chassis.getLLTx());

                        // Nota: Revisa si necesitas restar o sumar dependiendo de tu configuración
                        double target = heading - tx;

                        // 2. Le damos la orden al Follower
                        follower().turnTo(target);

                        ActiveOpMode.telemetry().addData("AutoAlign", "Targeting...");
                    } else {
                        ActiveOpMode.telemetry().addData("AutoAlign", "No Tag Found");
                    }
                }).
                setUpdate(
                        ()-> {}
                ).
                setStop(interrupted -> {
                    // Opcional: Si se interrumpe manualmente, frenar.
                    if(interrupted) chassis.stop();
                }).
                setIsDone(()-> {

                    return !follower().isBusy();
                }).
                setInterruptible(true);
                }

                //teleop apriltag not once
    public static Command alignWithJoysticks(SuperChassis chassis, DoubleSupplier forward, DoubleSupplier strafe) {
        return new LambdaCommand()
                .named("AlignWithJoysticks")
                .requires(chassis) // Takes control of the wheels
                .setStart(() -> {
                    // Optional: Switch pipeline or turn on light here
                    follower().startTeleOpDrive();
                })
                .setUpdate(() -> {
                    double turnPower = 0;

                    // 1. Check if we see a tag
                    if (chassis.isLLConnected() && chassis.getLIMELIGHT().getLatestResult().isValid()) {
                        // 2. Get the error (How far off center are we?)
                        double tx = chassis.getLLTx();

                        // 3. Calculate Turn Power (Proportional Controller)
                        // If tx is Negative (Left), we want Positive Turn (Left).
                        // So we invert tx. Start with kP = 0.02 and tune from there.
                        turnPower = -tx * 0.025;

                        // (Optional) Add a minimum power to overcome friction if close
                        if (Math.abs(tx) > 1.0 && Math.abs(turnPower) < 0.05) {
                            turnPower = Math.copySign(0.05, turnPower);
                        }

                        dev.nextftc.ftc.ActiveOpMode.telemetry().addData("AutoAlign", "Tracking... Err: %.2f", tx);
                    } else {
                        dev.nextftc.ftc.ActiveOpMode.telemetry().addData("AutoAlign", "NO TARGET");
                    }

                    // 4. Apply Power (Joysticks for XY, Limelight for Turn)
                    follower().setTeleOpDrive(forward.getAsDouble(), strafe.getAsDouble(), turnPower, false);
                })
                .setStop(interrupted -> chassis.stop())
                .setIsDone(() -> false) // Run forever until button is released
                .setInterruptible(true);
    }
}
