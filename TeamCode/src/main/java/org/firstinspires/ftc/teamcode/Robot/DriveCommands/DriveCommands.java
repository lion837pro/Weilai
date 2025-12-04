package org.firstinspires.ftc.teamcode.Robot.DriveCommands;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Lib.STZLite.Geometry.Rotation;
import org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Utils.Units;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.SuperChassis;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.VisionConstants;

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

    /**
     * Auto-align to AprilTag (autonomous style - completes when aligned).
     * ONLY aligns to tags 20 and 24 (alignment tags).
     */
    public static Command AlignByAprilTag(SuperChassis chassis){

        return new LambdaCommand().
                named("AlignByAprilTag").
                requires(chassis).
                setStart(()-> {}).
                setUpdate(()-> {
                    if(chassis.isLLConnected() && chassis.getLIMELIGHT().getLatestResult().isValid()) {
                        // Check if we see an alignment tag (ID 20 or 24)
                        int detectedId = chassis.getLastDetectedId();
                        if (!VisionConstants.isAlignmentTag(detectedId)) {
                            ActiveOpMode.telemetry().addData("AutoAlign", "Tag %d not alignment tag (need 20/24)", detectedId);
                            return;
                        }

                        double heading = chassis.getFOLLOWER().getPose().getHeading();
                        double tx = Math.toRadians(chassis.getLLTx());
                        double target = heading - tx;

                        follower().turnTo(target);

                        ActiveOpMode.telemetry().addData("AutoAlign", "Targeting Tag %d...", detectedId);
                    } else {
                        ActiveOpMode.telemetry().addData("AutoAlign", "No Tag Found");
                    }
                }).
                setStop(interrupted -> {
                    if(interrupted) chassis.stop();
                }).
                setIsDone(()-> {
                    return !follower().isBusy();
                }).
                setInterruptible(true);
    }

    /**
     * TeleOp auto-alignment with joystick movement.
     * ONLY aligns to tags 20 and 24 (alignment tags).
     * If no alignment tag visible, allows normal joystick control without auto-turn.
     */
    public static Command alignWithJoysticks(SuperChassis chassis, DoubleSupplier forward, DoubleSupplier strafe) {
        return new LambdaCommand()
                .named("AlignWithJoysticks")
                .requires(chassis)
                .setStart(() -> {
                    follower().startTeleOpDrive();
                })
                .setUpdate(() -> {
                    double turnPower = 0;

                    if (chassis.isLLConnected() && chassis.getLIMELIGHT().getLatestResult().isValid()) {
                        // Check if we see an alignment tag (ID 20 or 24)
                        int detectedId = chassis.getLastDetectedId();

                        if (VisionConstants.isAlignmentTag(detectedId)) {
                            // Valid alignment tag - do auto-align
                            double tx = chassis.getLLTx();
                            turnPower = -tx * 0.025;

                            if (Math.abs(tx) > 1.0 && Math.abs(turnPower) < 0.12) {
                                turnPower = Math.copySign(0.12, turnPower);
                            }

                            ActiveOpMode.telemetry().addData("AutoAlign", "Tag %d | Err: %.2f", detectedId, tx);
                        } else {
                            // Not an alignment tag - show info but don't auto-turn
                            ActiveOpMode.telemetry().addData("AutoAlign", "Tag %d (not alignment)", detectedId);
                        }
                    } else {
                        ActiveOpMode.telemetry().addData("AutoAlign", "NO TARGET");
                    }

                    follower().setTeleOpDrive(forward.getAsDouble(), strafe.getAsDouble(), turnPower, false);
                })
                .setStop(interrupted -> chassis.stop())
                .setIsDone(() -> false)
                .setInterruptible(true);
    }
}
