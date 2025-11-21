package org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.Intake;

import java.util.function.DoubleSupplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;

public class ShooterCommands {

//    public static Command runShooter(Shooter shooter, double power) {
//
//        return new LambdaCommand().
//                named("runShooter").
//                requires(shooter).
//                setStart(() -> {}).
//                setUpdate(() -> { shooter.set(power);}).
//                setStop(interrupted -> { shooter.set(0);
//                }).
//                setIsDone(() -> false).
//                setInterruptible(true);
//    }

    public static Command runManualShooter(Shooter shooter, DoubleSupplier powerSource) {
        return new LambdaCommand()
                .named("runManualShooter")
                .requires(shooter)
                .setStart(() -> {})
                .setUpdate(() -> {
                    // Read the value from the trigger/joystick every loop
                    double power = powerSource.getAsDouble();
                    shooter.set(power);
                })
                .setStop(interrupted -> shooter.set(0))
                .setIsDone(() -> false)
                .setInterruptible(true);
    }
    /**
     * Runs the shooter at a specific velocity using PID control.
     * @param shooter The shooter subsystem
     */

    public static Command shootWithFeed(Shooter shooter, Intake intake, double rpm) {
        return new ParallelGroup(
                // Command 1: Run the Shooter PID (Always running)
                runShooterPID(shooter, rpm),

                // Command 2: The "Smart Feeder"
                new LambdaCommand()
                        .named("SmartFeed")
                        .requires(intake) // Controls the intake
                        .setUpdate(() -> {
                            // ONLY run the intake if the shooter is ready
                            if (shooter.atSetpoint()) {
                                intake.MoveIn(1.0); // Feed the ball (adjust power if needed)
                            } else {
                                intake.MoveIn(0);   // Wait for shooter to speed up
                            }
                        })
                        .setStop(interrupted -> intake.MoveIn(0)) // Stop when button released
                        .setIsDone(() -> false)
                        .setInterruptible(true)
        );
    }
    public static Command runShooterPID(Shooter shooter, double rpm) {
        double targetTPS = ShooterConstants.rpmToTicksPerSecond(rpm);
        return new LambdaCommand()
                .named("runShooterPID")
                .requires(shooter)
                // Switch to PID mode and set the target
                .setStart(() -> shooter.toVelocity(targetTPS))
                // (Optional) We can keep setting it to ensure it stays in that mode
                .setUpdate(() -> shooter.toVelocity(targetTPS))
                // Stop when interrupted
                .setStop(interrupted -> shooter.stop())
                .setIsDone(() -> false) // Run forever until button released
                .setInterruptible(true);
    }

    public static Command stopShooter(Shooter shooter) {
        return new LambdaCommand()
                .named("stopShooter")
                .requires(shooter)
                // CHANGE '.Sh1(0)' TO '.set(0)' or '.stop()'
                .setStart(() -> shooter.stop())
                .setUpdate(() -> shooter.stop())
                .setIsDone(() -> true)
                .setInterruptible(true);
}
}
