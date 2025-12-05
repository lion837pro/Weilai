package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot.Hardware.REV312010;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.ChassisConstants;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.SuperChassis;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.IntakeCommands;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.LED.RobotFeedback;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.ShooterCommands;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer.Spindexer;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer.SpindexerCommands;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

/**
 * BLUEDOWN AUTONOMOUS
 *
 * Strategy:
 * 1. Score preloaded specimen on high chamber
 * 2. Collect 3 neutral samples from down zone
 * 3. Score samples in high basket
 * 4. Collect 3 neutral samples from mid zone
 * 5. Score samples in high basket
 * 6. Collect 3 neutral samples from up zone
 * 7. Score samples in high basket
 * 8. Park in observation zone
 */
@Autonomous(name = "Bluedown", group = "Competition")
public class Bluedown extends NextFTCOpMode {

    // Subsystems
    private final SuperChassis chassis = SuperChassis.INSTANCE;
    private final Intake intake = Intake.INSTANCE;
    private final Shooter shooter = Shooter.INSTANCE;
    private final Spindexer spindexer = Spindexer.INSTANCE;
    private REV312010 led;
    private RobotFeedback feedback;

    // Starting pose
    private final Pose startPose = new Pose(56, 8, Math.toRadians(0));

    // Path chains
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

    public Bluedown() {
        addComponents(new PedroComponent(ChassisConstants::buildPedroPathing));
        addComponents(chassis.asCOMPONENT());
        addComponents(intake.asCOMPONENT());
        addComponents(shooter.asCOMPONENT());
        addComponents(spindexer.asCOMPONENT());
    }

    /**
     * Build all path chains for the autonomous routine
     */
    public void buildPaths() {
        STbluealldown = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 8.000), new Pose(60.826, 14.786))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(118))
                .build();

        infrontofdownballs = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(60.826, 14.786), new Pose(40.495, 36.462))
                )
                .setLinearHeadingInterpolation(Math.toRadians(116), Math.toRadians(180))
                .build();

        intakedownballs = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(40.495, 36.462), new Pose(11.258, 36.294))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        shoot1point = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(11.258, 36.294), new Pose(66.539, 88.551))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        infrontmidballs = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(66.539, 88.551), new Pose(42.679, 59.818))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        intakemidballs = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(42.679, 59.818), new Pose(13.442, 59.482))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        conectionmidto2 = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(13.442, 59.482), new Pose(41.839, 71.076))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(160))
                .build();

        shooting2 = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(41.839, 71.076), new Pose(60.322, 86.702))
                )
                .setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(135))
                .build();

        infronofupballs = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(60.322, 86.702), new Pose(42.343, 83.174))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        intakeup = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(42.343, 83.174), new Pose(14.786, 83.678))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        shooting3 = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(14.786, 83.678), new Pose(65.699, 88.215))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        endpath = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(65.699, 88.215), new Pose(17.643, 13.106))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();
    }

    /**
     * Main autonomous routine using fully automated commands
     */
    public Command mainRoutine() {
        return new SequentialGroup(
                // ===== PHASE 1: Score Preload & Intake Down Balls =====
                // Navigate to high chamber
                new FollowPath(STbluealldown),
                ShooterCommands.shootAllBallsFixedRPM(shooter, spindexer, intake, 1600, feedback),
                new Delay(0.3),

                // Position in front of down balls
                new FollowPath(infrontofdownballs),

                // Intake down balls while moving (smart intake with spindexer)
                new ParallelGroup(
                        new FollowPath(intakedownballs),
                        IntakeCommands.intakeUntilFull(spindexer, intake, 0.8, feedback)
                ),

                // ===== PHASE 2: Score First Batch =====
                // Navigate to shooting position
                new FollowPath(shoot1point),

                // Shoot all balls with automated spindexer feeding
                ShooterCommands.shootAllBallsFixedRPM(shooter, spindexer, intake, 1600, feedback),

                // Small delay to ensure all balls shot
                new Delay(0.3),

                // ===== PHASE 3: Intake Mid Balls =====
                // Position in front of mid balls
                new FollowPath(infrontmidballs),

                // Intake mid balls while moving
                new ParallelGroup(
                        new FollowPath(intakemidballs),
                        IntakeCommands.intakeUntilFull(spindexer, intake, 0.8, feedback)
                ),

                // ===== PHASE 4: Score Second Batch =====
                // Connection path to shooting position
                new FollowPath(conectionmidto2),

                // Navigate to shooting position
                new FollowPath(shooting2),

                // Shoot all balls with automated spindexer feeding
                ShooterCommands.shootAllBallsFixedRPM(shooter, spindexer, intake, 1600, feedback),

                // Small delay to ensure all balls shot
                new Delay(0.3),

                // ===== PHASE 5: Intake Up Balls =====
                // Position in front of up balls
                new FollowPath(infronofupballs),

                // Intake up balls while moving
                new ParallelGroup(
                        new FollowPath(intakeup),
                        IntakeCommands.intakeUntilFull(spindexer, intake, 0.8, feedback)
                ),

                // ===== PHASE 6: Score Third Batch =====
                // Navigate to shooting position
                new FollowPath(shooting3),

                // Shoot all balls with automated spindexer feeding
                ShooterCommands.shootAllBallsFixedRPM(shooter, spindexer, intake, 1600, feedback),

                // Small delay to ensure all balls shot
                new Delay(0.3),

                // ===== PHASE 7: Park =====
                // Navigate to observation zone
                new FollowPath(endpath),

                // Stop all subsystems
                new SequentialGroup(
                        ShooterCommands.stopShooter(shooter),
                        IntakeCommands.stopIntake(intake),
                        SpindexerCommands.stopSpindexer(spindexer)
                )
        );
    }

    @Override
    public void onInit() {
        // Initialize feedback system
        try {
            led = new REV312010();
        } catch (Exception e) {
            led = null;
        }
        feedback = new RobotFeedback(led);

        // Set starting pose before building paths
        follower().setStartingPose(startPose);

        // Build the paths defined above
        buildPaths();

        // Optional: Home spindexer during init
        // SpindexerCommands.homeSpindexer(spindexer).invoke();

        telemetry.addData("Status", "Initialized - Ready to run!");
        telemetry.addData("Strategy", "Down → Mid → Up zones");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // Schedule the main routine to run
        mainRoutine().invoke();

        if (feedback != null) {
            feedback.setReady();
        }
    }

    @Override
    public void onUpdate() {
        // Update feedback system for LED strobing
        if (feedback != null) {
            feedback.update();
        }
    }

    @Override
    public void onStop() {
        // Clean shutdown
        if (feedback != null) {
            feedback.setIdle();
        }
    }
}
