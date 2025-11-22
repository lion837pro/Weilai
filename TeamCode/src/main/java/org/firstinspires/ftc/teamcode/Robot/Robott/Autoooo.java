//package org.firstinspires.ftc.teamcode.Robot.robot2;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.Robot.DriveCommands.DriveCommands;
//import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.Vision;
//import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.Intake;
//import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.IntakeCommands;
//import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.Shooter;
//import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.ShooterCommands;
//import org.firstinspires.ftc.teamcode.Robot.robot2.Chassiss;
//
//import dev.nextftc.core.commands.Command;
//import dev.nextftc.core.commands.delays.Delay;
//import dev.nextftc.core.commands.groups.ParallelGroup;
//import dev.nextftc.core.commands.groups.SequentialGroup;
//import dev.nextftc.core.units.Angle;
//import dev.nextftc.ftc.NextFTCOpMode;
//
//@Autonomous(name = "autoaksfdhlaskj")
//public class Autoooo extends NextFTCOpMode {
//
//    // Subsystems
//    private final Chassiss chassis = Chassiss.INSTANCE;
//    private final Vision vision = Vision.INSTANCE;
//    private final Intake intake = Intake.INSTANCE;
//    private final Shooter shooter = Shooter.INSTANCE;
//
//    public Autoooo() {
//        // Add subsystem components
//        addComponents(chassis.asCOMPONENT());
//        addComponents(vision.asCOMPONENT());
//        addComponents(intake.asCOMPONENT());
//        addComponents(shooter.asCOMPONENT());
//    }
//
//    /**
//     * Define the main autonomous routine
//     */
//    public Command mainRoutine() {
//        return new SequentialGroup(
//                // Step 1: Drive forward and align to tag
//                new ParallelGroup(
//                        DriveCommands.driveDistance(chassis, 24, 0.5),
//                        DriveCommands.autoAlignToTag(chassis, vision)
//                ),
//
//                // Step 2: Wait a moment
//                new Delay(0.5),
//
//                // Step 3: Spin up shooter and shoot
//                new ParallelGroup(
//                        ShooterCommands.autoRevShooter(shooter, vision),
//                        new SequentialGroup(
//                                new Delay(1.0), // Wait for shooter to spin up
//                                IntakeCommands.runIntake(intake, 1.0),
//                                new Delay(2.0), // Feed for 2 seconds
//                                IntakeCommands.stopIntake(intake)
//                        )
//                ),
//
//                // Step 4: Turn 90 degrees
//                DriveCommands.turnBy(chassis, 90),
//
//                // Step 5: Drive forward again
//                DriveCommands.driveDistance(chassis, 12, 0.5),
//
//                // Step 6: Stop everything
//                new ParallelGroup(
//                        ShooterCommands.stopShooter(shooter),
//                        IntakeCommands.stopIntake(intake)
//                )
//        );
//    }
//
//    @Override
//    public void onInit() {
//        telemetry.addLine("=== SIMPLE AUTONOMOUS ===");
//        telemetry.addLine();
//        telemetry.addLine("This routine will:");
//        telemetry.addLine("1. Drive forward 24 inches");
//        telemetry.addLine("2. Auto-align to AprilTag");
//        telemetry.addLine("3. Shoot with auto-aim");
//        telemetry.addLine("4. Turn 90 degrees");
//        telemetry.addLine("5. Drive forward 12 inches");
//        telemetry.addLine();
//        telemetry.addLine("Press START when ready");
//        telemetry.update();
//    }
//
//    @Override
//    public void onStartButtonPressed() {
//        // Schedule the main routine
//        mainRoutine().schedule();
//    }
//}