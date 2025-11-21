package org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.Intake;

import java.util.function.DoubleSupplier;

import dev.nextftc.core.commands.Command;
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
     * @param velocity The target velocity in Ticks Per Second (e.g., 1500)
     */
    public static Command runShooterPID(Shooter shooter, double velocity) {
        return new LambdaCommand()
                .named("runShooterPID")
                .requires(shooter)
                // Switch to PID mode and set the target
                .setStart(() -> shooter.toVelocity(velocity))
                // (Optional) We can keep setting it to ensure it stays in that mode
                .setUpdate(() -> shooter.toVelocity(velocity))
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
