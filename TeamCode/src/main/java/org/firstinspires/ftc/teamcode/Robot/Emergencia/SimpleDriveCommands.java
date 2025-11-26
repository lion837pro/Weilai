package org.firstinspires.ftc.teamcode.Robot.Emergencia;

import org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Controller.PIDController;

import java.util.function.DoubleSupplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.ftc.ActiveOpMode;

public class SimpleDriveCommands {

    /**
     * Standard teleop drive command with joystick control
     */
    public static Command driveWithJoysticks(
            SimpleMecanumChassis chassis,
            DoubleSupplier forward,
            DoubleSupplier strafe,
            DoubleSupplier turn) {

        return new LambdaCommand()
                .named("DriveWithJoysticks")
                .requires(chassis)
                .setUpdate(() -> {
                    double fw = forward.getAsDouble();
                    double st = strafe.getAsDouble();
                    double tr = turn.getAsDouble();

                    // Apply deadband
                    if (Math.abs(fw) < 0.05) fw = 0;
                    if (Math.abs(st) < 0.05) st = 0;
                    if (Math.abs(tr) < 0.05) tr = 0;

                    chassis.drive(fw, st, tr);
                })
                .setStop(interrupted -> chassis.stop())
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    /**
     * Drive with joysticks but auto-align to AprilTag
     */
    public static Command alignedDrive(
            SimpleMecanumChassis chassis,
            SimpleVision vision,
            DoubleSupplier forward,
            DoubleSupplier strafe) {

        PIDController alignPID = new PIDController(0.03, 0, 0.001);
        alignPID.setTolerance(2.0);

        return new LambdaCommand()
                .named("AlignedDrive")
                .requires(chassis)
                .setStart(() -> {
                    alignPID.reset();
                    chassis.setBrakeMode();
                })
                .setUpdate(() -> {
                    double turnPower = 0;

                    if (vision.hasTarget()) {
                        double error = vision.getAlignmentError();
                        turnPower = alignPID.calculate(error, 0).getAsDouble();

                        // Clamp turn power
                        turnPower = Math.max(-0.5, Math.min(0.5, turnPower));

                        // Add minimum power to overcome friction
                        if (Math.abs(error) > 1.0 && Math.abs(turnPower) < 0.1) {
                            turnPower = Math.copySign(0.1, turnPower);
                        }

                        ActiveOpMode.telemetry().addData("Auto-Align", "Active (Error: %.1f)", error);
                    } else {
                        ActiveOpMode.telemetry().addData("Auto-Align", "No Target");
                    }

                    chassis.drive(forward.getAsDouble(), strafe.getAsDouble(), turnPower);
                })
                .setStop(interrupted -> {
                    chassis.stop();
                    alignPID.reset();
                })
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    /**
     * Turn to align with AprilTag (autonomous style - completes when aligned)
     */
    public static Command alignToTag(SimpleMecanumChassis chassis, SimpleVision vision) {
        PIDController alignPID = new PIDController(0.03, 0, 0.001);
        alignPID.setTolerance(1.0);

        return new LambdaCommand()
                .named("AlignToTag")
                .requires(chassis)
                .setStart(() -> {
                    alignPID.reset();
                    chassis.setBrakeMode();
                })
                .setUpdate(() -> {
                    if (vision.hasTarget()) {
                        double error = vision.getAlignmentError();
                        double turnPower = alignPID.calculate(error, 0).getAsDouble();

                        turnPower = Math.max(-0.5, Math.min(0.5, turnPower));

                        if (Math.abs(error) > 1.0 && Math.abs(turnPower) < 0.1) {
                            turnPower = Math.copySign(0.1, turnPower);
                        }

                        chassis.drive(0, 0, turnPower);

                        ActiveOpMode.telemetry().addData("Aligning", "Error: %.1f deg", error);
                    } else {
                        chassis.stop();
                        ActiveOpMode.telemetry().addData("Aligning", "Lost Target!");
                    }
                })
                .setStop(interrupted -> {
                    chassis.stop();
                    alignPID.reset();
                })
                .setIsDone(() -> {
                    return !vision.hasTarget() || alignPID.atSetpoint();
                })
                .setInterruptible(true);
    }

    /**
     * Reset heading to zero
     */
    public static Command resetHeading(SimpleMecanumChassis chassis) {
        return new InstantCommand("ResetHeading", chassis::resetHeading);
    }

    /**
     * Toggle field-centric mode
     */
    public static Command toggleFieldCentric(SimpleMecanumChassis chassis) {
        return new InstantCommand("ToggleFieldCentric", () -> {
            chassis.toggleFieldCentric();
            ActiveOpMode.telemetry().addData("Field-Centric", chassis.isFieldCentric() ? "ON" : "OFF");
        });
    }

    /**
     * Set slow mode (reduces power to 30%)
     */
    public static Command enableSlowMode(SimpleMecanumChassis chassis) {
        return new InstantCommand("SlowMode", () -> chassis.setPowerMultiplier(0.3));
    }

    /**
     * Set normal mode (full power)
     */
    public static Command enableNormalMode(SimpleMecanumChassis chassis) {
        return new InstantCommand("NormalMode", () -> chassis.setPowerMultiplier(1.0));
    }

    /**
     * Toggle between slow and normal mode
     */
    public static Command toggleSpeed(SimpleMecanumChassis chassis) {
        return new InstantCommand("ToggleSpeed", () -> {
            double current = chassis.getPowerMultiplier();
            if (current > 0.5) {
                chassis.setPowerMultiplier(0.3);
            } else {
                chassis.setPowerMultiplier(1.0);
            }
        });
    }

    /**
     * Drive forward at constant speed
     */
    public static Command driveForward(SimpleMecanumChassis chassis, double speed) {
        return new LambdaCommand()
                .named("DriveForward")
                .requires(chassis)
                .setUpdate(() -> chassis.drive(speed, 0, 0))
                .setStop(interrupted -> chassis.stop())
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    /**
     * Strafe at constant speed
     */
    public static Command strafe(SimpleMecanumChassis chassis, double speed) {
        return new LambdaCommand()
                .named("Strafe")
                .requires(chassis)
                .setUpdate(() -> chassis.drive(0, speed, 0))
                .setStop(interrupted -> chassis.stop())
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    /**
     * Turn at constant speed
     */
    public static Command turn(SimpleMecanumChassis chassis, double speed) {
        return new LambdaCommand()
                .named("Turn")
                .requires(chassis)
                .setUpdate(() -> chassis.drive(0, 0, speed))
                .setStop(interrupted -> chassis.stop())
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    /**
     * Turn to a specific heading
     */
    public static Command turnToHeading(SimpleMecanumChassis chassis, double targetDegrees) {
        PIDController headingPID = new PIDController(0.02, 0, 0.001);
        headingPID.setTolerance(2.0);
        headingPID.enableContinuousInput(-180, 180);

        return new LambdaCommand()
                .named("TurnToHeading")
                .requires(chassis)
                .setStart(() -> {
                    headingPID.reset();
                    headingPID.setSetpoint(targetDegrees);
                    chassis.setBrakeMode();
                })
                .setUpdate(() -> {
                    double currentHeading = chassis.getHeading();
                    double turnPower = headingPID.calculate(currentHeading).getAsDouble();

                    turnPower = Math.max(-0.6, Math.min(0.6, turnPower));

                    chassis.drive(0, 0, turnPower);

                    ActiveOpMode.telemetry().addData("Target Heading", "%.1f deg", targetDegrees);
                    ActiveOpMode.telemetry().addData("Current Heading", "%.1f deg", currentHeading);
                    ActiveOpMode.telemetry().addData("Error", "%.1f deg", headingPID.getError());
                })
                .setStop(interrupted -> {
                    chassis.stop();
                    headingPID.reset();
                })
                .setIsDone(headingPID::atSetpoint)
                .setInterruptible(true);
    }

    /**
     * Turn by a relative angle
     */
    public static Command turnBy(SimpleMecanumChassis chassis, double angleDegrees) {
        return new LambdaCommand()
                .named("TurnBy")
                .requires(chassis)
                .setStart(() -> {
                    double currentHeading = chassis.getHeading();
                    double targetHeading = currentHeading + angleDegrees;

                    // Normalize to -180 to 180
                    while (targetHeading > 180) targetHeading -= 360;
                    while (targetHeading < -180) targetHeading += 360;

                    turnToHeading(chassis, targetHeading).schedule();
                })
                .setIsDone(() -> true)
                .setInterruptible(true);
    }

    /**
     * Stop the chassis
     */
    public static Command stop(SimpleMecanumChassis chassis) {
        return new InstantCommand("Stop", chassis::stop);
    }
}