package org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.SuperChassis;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.VisionConstants;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.Intake;

import java.util.function.DoubleSupplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;

public class ShooterCommands {

    //helper commands
    public static Command smartFeed(Shooter shooter, Intake intake) {
        return new LambdaCommand()
                .named("SmartFeed")
                .requires(intake) // Takes control of Intake
                .setUpdate(() -> {
                    // 'atSetpoint()' checks if we reached the target
                    // (even if that target is constantly changing!)
                    if (shooter.atSetpoint()) {
                        intake.MoveIn(1.0);// Fire!
                    } else {
                        intake.MoveIn(0);    // Wait for spin-up
                    }
                })
                .setStop(interrupted -> intake.MoveIn(0))
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    public static Command runManualShooter(Shooter shooter, DoubleSupplier powerSource) {
        return new LambdaCommand()
                .named("runManualShooter")
                .requires(shooter)
                .setStart(() -> {
                })
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
     *
     * @param shooter The shooter subsystem
     */
//low level commands
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

    public static Command autoRevShooter(Shooter shooter, SuperChassis chassis) {
        return new LambdaCommand()
                .named("autoRevShooter")
                .requires(shooter)
                .setUpdate(() -> {
                    // 1. Get Distance
                    double distance = chassis.getDistanceToTag();

                    // 2. Calculate RPM (Linear Equation)
                    // RPM = Base + (Inches * Slope)
                    // Example: 2000 + (50 * distance)
                    double targetRPM = VisionConstants.BASE_RPM + (distance * VisionConstants.RPM_PER_INCH);

                    // Safety Clamp (Don't go over motor max)
                    if (targetRPM > ShooterConstants.MAX_RPM) targetRPM = ShooterConstants.MAX_RPM;
                    if (distance <= 0) targetRPM = 2000; // Default if no tag seen

                    // 3. Convert and Set
                    double targetTPS = ShooterConstants.rpmToTicksPerSecond(targetRPM);
                    shooter.toVelocity(targetTPS);

                    // Optional: Debug
                    dev.nextftc.ftc.ActiveOpMode.telemetry().addData("AutoAim Dist", "%.1f in", distance);
                    dev.nextftc.ftc.ActiveOpMode.telemetry().addData("AutoAim RPM", "%.0f", targetRPM);
                })
                .setStop(interrupted -> shooter.stop())
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    //auto shoots commands
    public static Command shootWithFeed(Shooter shooter, Intake intake, double rpm) {
        return new ParallelGroup(
                // Command 1: Run the Shooter PID (Always running)
                runShooterPID(shooter, rpm),
                smartFeed(shooter, intake)
                // Command 2: The "Smart Feeder
        );
    }
    public static Command shootWithAutoAim(Shooter shooter, Intake intake, SuperChassis chassis) {
        return new ParallelGroup(
                // Command A: Run the Auto-Rev Logic (Reuse your existing command!)
                autoRevShooter(shooter, chassis),
                smartFeed(shooter, intake)
                // Command B: The Smart Feeder
        );
    }

}