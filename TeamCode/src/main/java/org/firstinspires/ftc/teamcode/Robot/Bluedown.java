package org.firstinspires.ftc.teamcode.Robot;
import static org.firstinspires.ftc.teamcode.Lib.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.SuperChassis;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.ChassisConstants;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.IntakeCommands;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.ShooterCommands;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;


@Autonomous(name = "Bluedown", group = "Templates")
public class Bluedown extends NextFTCOpMode {

    // 1. Declare Subsystems and Components
    private final SuperChassis chassis = SuperChassis.INSTANCE;

    private final Intake intake = Intake.INSTANCE;
    private final Shooter shooter = Shooter.INSTANCE;


    private final Pose startPose = new Pose(56, 8, Math.toRadians(0));


    public Bluedown() {
        addComponents(new PedroComponent(ChassisConstants::buildPedroPathing));
        addComponents(chassis.asCOMPONENT());
        addComponents(intake.asCOMPONENT());
        addComponents(shooter.asCOMPONENT());

    }
    private PathChain STbluealldown;
    private PathChain infrontofdownballs;
    private PathChain intakedownballs;
    private PathChain shoot1point;
    private PathChain infrontmidballs;
    private PathChain intakemidballs;
    private PathChain conectionmidto2;
    private PathChain shooting2;
    private PathChain infronofupballs;
    private PathChain intakeup;
    private PathChain shooting3;
    private PathChain endpath;

    public void buildPaths() {
                STbluealldown = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(56.000, 8.000), new Pose(60.826, 14.786))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(118))
                        .build();

                infrontofdownballs = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(60.826, 14.786), new Pose(40.495, 36.462))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(116), Math.toRadians(180))
                        .build();

                intakedownballs = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(40.495, 36.462), new Pose(11.258, 36.294))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                        .build();

                shoot1point = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(11.258, 36.294), new Pose(66.539, 88.551))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                        .build();

                infrontmidballs = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(66.539, 88.551), new Pose(42.679, 59.818))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                        .build();

                intakemidballs = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(42.679, 59.818), new Pose(13.442, 59.482))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                        .build();

                conectionmidto2 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(13.442, 59.482), new Pose(41.839, 71.076))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(160))
                        .build();

                shooting2 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(41.839, 71.076), new Pose(60.322, 86.702))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(135))
                        .build();

                infronofupballs = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(60.322, 86.702), new Pose(42.343, 83.174))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                        .build();

                intakeup = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(42.343, 83.174), new Pose(14.786, 83.678))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                        .build();

                shooting3 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(14.786, 83.678), new Pose(65.699, 88.215))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                        .build();

                endpath = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(65.699, 88.215), new Pose(17.643, 13.106))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                        .build();
            }



    /**
     * Define the main sequence of commands for the routine.
     */
    public Command mainRoutine() {
        return new SequentialGroup(
        new SequentialGroup(
                new FollowPath(STbluealldown),
                new FollowPath(infrontofdownballs)
        ),
        new ParallelGroup(
                new FollowPath(intakedownballs), IntakeCommands.runIntake(intake,0.6)
        ),
        new Delay(.5),
            IntakeCommands.stopIntake(intake),
        new ParallelGroup(
                new FollowPath(shoot1point)),
        new Delay(.5),
                ShooterCommands.shootWithFeed(shooter,intake,1800),
        new ParallelGroup(
                new FollowPath(infrontmidballs), ShooterCommands.stopShooter(shooter)
        ));
    };

    @Override
    public void onInit() {
        // Set starting pose before building paths
        follower.setStartingPose(startPose);

        // Build the paths defined above
        buildPaths();
    };

    @Override
    public void onStartButtonPressed() {
        // Schedule the main routine to run
        mainRoutine().invoke();
    }
}