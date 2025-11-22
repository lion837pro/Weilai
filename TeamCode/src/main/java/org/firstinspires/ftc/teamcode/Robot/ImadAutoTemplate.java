package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.geometry.Pose;
//import org.firstinspires.ftc.teamcode.Lib.STZLite.Geometry.Pose;
import org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Utils.Units;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.SuperChassis;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

@Autonomous(name = "NextFTC Autonomous Program 2 Java")
public class ImadAutoTemplate extends NextFTCOpMode {

    private final SuperChassis chassis = SuperChassis.INSTANCE;


    public ImadAutoTemplate() {
        addComponents(chassis.asCOMPONENT());
    }

    private final Pose startPose = new Pose(9.0, 60.0, Math.toRadians(Units.degreesToRadians(0)));
    private final Pose finishPose = new Pose(37.0, 50.0, Math.toRadians(Units.degreesToRadians(180)));


    private PathChain move;

    public void buildPaths() {
        move = follower().pathBuilder()
                .addPath(new BezierLine(startPose, finishPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), finishPose.getHeading()).build();


    }

    public Command mainRoutine() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(move)
                ),
                new ParallelGroup(
                ),
                new Delay(1.0)
        );
    }

    @Override
    public void onInit() {
        buildPaths();
    }

    @Override
    public void onStartButtonPressed() {
        mainRoutine().invoke();
    }
}