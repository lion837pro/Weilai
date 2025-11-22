package org.firstinspires.ftc.teamcode.Robot.Robott.Commands2;

import org.firstinspires.ftc.teamcode.Lib.STZLite.Drive.Chassis;
import org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Controller.PIDController;
import org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Utils.MathUtil;
import org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Utils.Units;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.ChassisConstants;
import org.firstinspires.ftc.teamcode.Robot.Robott.Subsystems2.Vision;

import java.util.function.DoubleSupplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.units.Angle;
import dev.nextftc.ftc.ActiveOpMode;

public class ChassisCommands {

    // ========== TELEOP COMMANDS ==========

    /**
     * Standard teleop drive command with joystick control
     * @param chassis The Chassis subsystem
     * @param forward Forward/backward supplier (-1 to 1)
     * @param strafe Left/right supplier (-1 to 1)
     * @param turn Rotation supplier (-1 to 1)
     * @param fieldCentric Whether to use field-centric control
     */
    public static Command driveWithJoysticks(
            Chassis chassis,
            DoubleSupplier forward,
            DoubleSupplier strafe,
            DoubleSupplier turn,
            boolean fieldCentric) {

        return new LambdaCommand()
                .named("DriveWithJoysticks")
                .requires(chassis)
                .setStart(() -> {})
                .setUpdate(() -> {
                    double fw = forward.getAsDouble();
                    double st = strafe.getAsDouble();
                    double tr = turn.getAsDouble();

                    chassis.drive(fw, st, tr, fieldCentric);
                })
                .setStop(interrupted -> chassis.stop())
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    /**
     * Drive with joysticks but auto-align heading to AprilTag using Vision
     * @param chassis The Chassis subsystem
     * @param vision The Vision subsystem
     * @param forward Forward/backward supplier
     * @param strafe Left/right supplier
     */
    public static Command alignWithJoysticks(
            Chassis chassis,
            Vision vision,
            DoubleSupplier forward,
            DoubleSupplier strafe) {

        // PID controller for turn alignment
        PIDController turnPID = new PIDController(0.025, 0, 0.002);
        turnPID.setTolerance(2.0); // 2 degree tolerance

        return new LambdaCommand()
                .named("AlignWithJoysticks")
                .requires(chassis)
                .setStart(() -> {
                    chassis.brake();
                })
                .setUpdate(() -> {
                    double turnPower = 0;

                    // Check if we see a tag
                    if (vision.isConnected() && vision.hasTarget()) {
                        // Get horizontal error
                        double tx = vision.getTx();

                        // Calculate turn power using PID
                        turnPower = -turnPID.calculate(tx, 0).getAsDouble();

                        // Clamp turn power
                        turnPower = MathUtil.clamp(turnPower, -0.5, 0.5);

                        // Add minimum power to overcome friction
                        if (Math.abs(tx) > 1.0 && Math.abs(turnPower) < 0.12) {
                            turnPower = Math.copySign(0.12, turnPower);
                        }

                        ActiveOpMode.telemetry().addData("AutoAlign", "Tracking... Err: %.2f", tx);
                    } else {
                        ActiveOpMode.telemetry().addData("AutoAlign", "NO TARGET");
                    }

                    // Drive with manual XY, auto turn
                    chassis.drive(forward.getAsDouble(), strafe.getAsDouble(), turnPower, false);
                })
                .setStop(interrupted -> {
                    chassis.stop();
                    turnPID.reset();
                })
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    // ========== UTILITY COMMANDS ==========

    /**
     * Reset the heading to zero
     */
    public static Command resetHeading(Chassis chassis) {
        return new InstantCommand("ResetHeading", chassis::resetHeading);
    }

    /**
     * Toggle field-centric mode
     */
    public static Command toggleFieldCentric(Chassis chassis) {
        return new InstantCommand("ToggleFieldCentric", () -> {
            chassis.setFieldCentric(!chassis.isFieldCentric());
        });
    }

    /**
     * Set brake mode
     */
    public static Command setBrakeMode(Chassis chassis) {
        return new InstantCommand("SetBrakeMode", chassis::setBrakeMode);
    }

    /**
     * Set coast mode
     */
    public static Command setCoastMode(Chassis chassis) {
        return new InstantCommand("SetCoastMode", chassis::setCoastMode);
    }

    // ========== AUTONOMOUS TURN COMMANDS ==========

    /**
     * Turn to a specific heading (absolute angle)
     * @param chassis The Chassis subsystem
     * @param targetHeading Target heading as Angle
     */
    public static Command turnToHeading(Chassis chassis, Angle targetHeading) {
        PIDController headingPID = new PIDController(
                ChassisConstants.HEADING_KP,
                ChassisConstants.HEADING_KI,
                ChassisConstants.HEADING_KD
        );

        headingPID.setTolerance(Units.degreesToRadians(ChassisConstants.HEADING_TOLERANCE_DEGREES));
        headingPID.enableContinuousInput(-Math.PI, Math.PI);

        return new LambdaCommand()
                .named("TurnToHeading")
                .requires(chassis)
                .setStart(() -> {
                    chassis.setBrakeMode();
                    headingPID.setSetpoint(targetHeading.inRad);
                })
                .setUpdate(() -> {
                    double currentHeading = chassis.getHeading().inRad;
                    double turnPower = headingPID.calculate(currentHeading).getAsDouble();

                    // Clamp power
                    turnPower = MathUtil.clamp(turnPower, -ChassisConstants.MAX_TURN_SPEED, ChassisConstants.MAX_TURN_SPEED);

                    chassis.drive(0, 0, turnPower, false);

                    ActiveOpMode.telemetry().addData("Target Heading", "%.2f deg", Units.radiansToDegrees(targetHeading.inRad));
                    ActiveOpMode.telemetry().addData("Current Heading", "%.2f deg", Units.radiansToDegrees(currentHeading));
                    ActiveOpMode.telemetry().addData("Error", "%.2f deg", Units.radiansToDegrees(headingPID.getError()));
                })
                .setStop(interrupted -> {
                    chassis.stop();
                    headingPID.reset();
                })
                .setIsDone(headingPID::atSetpoint)
                .setInterruptible(true);
    }

    /**
     * Turn to a specific heading (degrees)
     */
    public static Command turnToHeading(Chassis chassis, double headingDegrees) {
        return turnToHeading(chassis, Angle.fromDeg(headingDegrees));
    }

    /**
     * Turn by a relative angle
     * @param chassis The Chassis subsystem
     * @param deltaHeading Angle to turn by
     */
    public static Command turnBy(Chassis chassis, Angle deltaHeading) {
        return new LambdaCommand()
                .named("TurnBy")
                .requires(chassis)
                .setStart(() -> {
                    double currentHeading = chassis.getHeading().inRad;
                    double targetHeading = currentHeading + deltaHeading.inRad;

                    // Schedule the turn to heading command
                    turnToHeading(chassis, Angle.fromRad(targetHeading)).schedule();
                })
                .setUpdate(() -> {})
                .setStop(interrupted -> {})
                .setIsDone(() -> true)
                .setInterruptible(true);
    }

    /**
     * Turn by a relative angle (degrees)
     */
    public static Command turnBy(Chassis chassis, double deltaHeadingDegrees) {
        return turnBy(chassis, Angle.fromDeg(deltaHeadingDegrees));
    }

    // ========== VISION-BASED AUTONOMOUS COMMANDS ==========

    /**
     * Auto-align to AprilTag (autonomous version - completes when aligned)
     * @param chassis The Chassis subsystem
     * @param vision The Vision subsystem
     */
    public static Command autoAlignToTag(Chassis chassis, Vision vision) {
        PIDController turnPID = new PIDController(0.025, 0, 0.002);
        turnPID.setTolerance(1.0); // 1 degree tolerance

        return new LambdaCommand()
                .named("AutoAlignToTag")
                .requires(chassis)
                .setStart(() -> {
                    chassis.setBrakeMode();
                    turnPID.reset();
                })
                .setUpdate(() -> {
                    if (vision.isConnected() && vision.hasTarget()) {
                        double tx = vision.getTx();
                        double turnPower = -turnPID.calculate(tx, 0).getAsDouble();

                        turnPower = MathUtil.clamp(turnPower, -0.5, 0.5);

                        // Minimum power
                        if (Math.abs(tx) > 1.0 && Math.abs(turnPower) < 0.12) {
                            turnPower = Math.copySign(0.12, turnPower);
                        }

                        chassis.drive(0, 0, turnPower, false);

                        ActiveOpMode.telemetry().addData("AutoAlign", "Aligning... Err: %.2f", tx);
                    } else {
                        chassis.stop();
                        ActiveOpMode.telemetry().addData("AutoAlign", "NO TARGET - STOPPING");
                    }
                })
                .setStop(interrupted -> {
                    chassis.stop();
                    turnPID.reset();
                })
                .setIsDone(() -> {
                    // Done when aligned OR no target
                    return !vision.hasTarget() || turnPID.atSetpoint();
                })
                .setInterruptible(true);
    }

    /**
     * Drive forward while auto-aligning to AprilTag
     * @param chassis The Chassis subsystem
     * @param vision The Vision subsystem
     * @param speed Forward speed (0 to 1)
     */
    public static Command driveAndAlign(Chassis chassis, Vision vision, double speed) {
        PIDController turnPID = new PIDController(0.025, 0, 0.002);
        turnPID.setTolerance(2.0);

        return new LambdaCommand()
                .named("DriveAndAlign")
                .requires(chassis)
                .setStart(() -> {
                    chassis.setBrakeMode();
                    turnPID.reset();
                })
                .setUpdate(() -> {
                    double turnPower = 0;

                    if (vision.isConnected() && vision.hasTarget()) {
                        double tx = vision.getTx();
                        turnPower = -turnPID.calculate(tx, 0).getAsDouble();
                        turnPower = MathUtil.clamp(turnPower, -0.5, 0.5);

                        if (Math.abs(tx) > 1.0 && Math.abs(turnPower) < 0.12) {
                            turnPower = Math.copySign(0.12, turnPower);
                        }
                    }

                    chassis.drive(speed, 0, turnPower, false);
                })
                .setStop(interrupted -> {
                    chassis.stop();
                    turnPID.reset();
                })
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    /**
     * Strafe while auto-aligning to AprilTag
     * @param chassis The Chassis subsystem
     * @param vision The Vision subsystem
     * @param speed Strafe speed (0 to 1)
     */
    public static Command strafeAndAlign(Chassis chassis, Vision vision, double speed) {
        PIDController turnPID = new PIDController(0.025, 0, 0.002);
        turnPID.setTolerance(2.0);

        return new LambdaCommand()
                .named("StrafeAndAlign")
                .requires(chassis)
                .setStart(() -> {
                    chassis.setBrakeMode();
                    turnPID.reset();
                })
                .setUpdate(() -> {
                    double turnPower = 0;

                    if (vision.isConnected() && vision.hasTarget()) {
                        double tx = vision.getTx();
                        turnPower = -turnPID.calculate(tx, 0).getAsDouble();
                        turnPower = MathUtil.clamp(turnPower, -0.5, 0.5);

                        if (Math.abs(tx) > 1.0 && Math.abs(turnPower) < 0.12) {
                            turnPower = Math.copySign(0.12, turnPower);
                        }
                    }

                    chassis.drive(0, speed, turnPower, false);
                })
                .setStop(interrupted -> {
                    chassis.stop();
                    turnPID.reset();
                })
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    // ========== BASIC DRIVE COMMANDS ==========

    /**
     * Drive forward at a constant speed
     * @param chassis The Chassis subsystem
     * @param speed Forward speed (-1 to 1)
     */
    public static Command driveForward(Chassis chassis, double speed) {
        return new LambdaCommand()
                .named("DriveForward")
                .requires(chassis)
                .setStart(() -> chassis.setBrakeMode())
                .setUpdate(() -> chassis.drive(speed, 0, 0, false))
                .setStop(interrupted -> chassis.stop())
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    /**
     * Strafe at a constant speed
     * @param chassis The Chassis subsystem
     * @param speed Strafe speed (-1 to 1)
     */
    public static Command strafe(Chassis chassis, double speed) {
        return new LambdaCommand()
                .named("Strafe")
                .requires(chassis)
                .setStart(() -> chassis.setBrakeMode())
                .setUpdate(() -> chassis.drive(0, speed, 0, false))
                .setStop(interrupted -> chassis.stop())
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    /**
     * Turn in place at a constant speed
     * @param chassis The Chassis subsystem
     * @param speed Turn speed (-1 to 1)
     */
    public static Command turnInPlace(Chassis chassis, double speed) {
        return new LambdaCommand()
                .named("TurnInPlace")
                .requires(chassis)
                .setStart(() -> chassis.setBrakeMode())
                .setUpdate(() -> chassis.drive(0, 0, speed, false))
                .setStop(interrupted -> chassis.stop())
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    /**
     * Stop the chassis
     */
    public static Command stop(Chassis chassis) {
        return new InstantCommand("Stop", chassis::stop);
    }
}