package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.ChassisConstants;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.SuperChassis;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.IntakeCommands;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.ShooterCommands;
// Add other subsystems if needed (Intake, Shooter)
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

@Autonomous(name = "GeminiAutoTemplate", group = "Templates")
public class GeminiAutoTemplate extends NextFTCOpMode {

    // 1. Declare Subsystems and Components
    private final SuperChassis chassis = SuperChassis.INSTANCE;
    private final Intake intake = Intake.INSTANCE;
    private final Shooter shooter = Shooter.INSTANCE;

    // Add other subsystems here if you want to use them in commands
    // private final Intake intake = Intake.INSTANCE;

    // 2. Define Starting Pose
    // TODO: Set your actual starting position here (in inches and radians)
    // Example: Starting at (9, 60) facing 0 degrees
    private final Pose startPose = new Pose(9, 60, Math.toRadians(0));

    // 3. Declare PathChains
    private PathChain apple;
    private PathChain banana; // Example for parallel

    public GeminiAutoTemplate() {
        // 4. Initialize PedroComponent FIRST
        addComponents(new PedroComponent(ChassisConstants::buildPedroPathing));
        addComponents(chassis.asCOMPONENT());
        addComponents(intake.asCOMPONENT());
        addComponents(shooter.asCOMPONENT());

    }

    /**
     * Build all paths for this autonomous routine.
     * This is called in onInit().
     */


    public void buildPaths() {
        // --- PASTE VISUALIZER CODE HERE ---

        // Example 1: "apple" Path
        apple = follower().pathBuilder()
                .addPath(
                        // Note: Use new Point(x, y, Point.CARTESIAN) for best compatibility
                        new BezierLine(new Pose(56.000, 8.000), new Pose(60.826, 14.78))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(118))
                .build();

        // Example 2: "banana" Path (for the parallel example)

        banana = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 8.000), new Pose(60.826, 14.786))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(118))
                .build();
    }

    /**
     * Define the main sequence of commands for the routine.
     */
    public Command mainRoutine() {
        return new SequentialGroup(
                // Step 1: Follow the first path
                new FollowPath(apple),

                // Step 2: Wait a bit
                new Delay(0.5),

                // Step 3: Example Parallel Command Group
                // This runs multiple things AT THE SAME TIME
                new ParallelGroup(
                        // Action A: Drive the second path
                        new FollowPath(banana),

                        // Action B: Do something else while driving (e.g., Run Intake)
                        // IntakeCommands.runIntake(intake, 1.0)

                        // Action C: Move Shooter (e.g., Rev up)
                        // ShooterCommands.runShooterPID(shooter, 3000)

                        // Just a print for example if subsystems aren't ready
                        new dev.nextftc.core.commands.utility.InstantCommand("", () -> telemetry.addData("Status", "Driving and Shooting!"))
                ),

                // Step 4: Stop everything at the end
                new ParallelGroup(
                        // ShooterCommands.stopShooter(shooter),
                        // IntakeCommands.stopIntake(intake)
                )
        );
    }

    @Override
    public void onInit() {
        // Set starting pose BEFORE building paths
        follower().setStartingPose(startPose);

        // Build the paths defined above
        buildPaths();
    }

    @Override
    public void onStartButtonPressed() {
        // Schedule the main routine to run
        mainRoutine().schedule();
    }
}