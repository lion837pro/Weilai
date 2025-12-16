package org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.SuperChassis;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.VisionConstants;

import java.util.function.DoubleSupplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;

/**
 * Commands for the Turret subsystem.
 * Includes manual control, preset positions, and vision-based auto-alignment.
 */
public class TurretCommands {

    // ===== MANUAL CONTROL =====

    /**
     * Manual turret control with joystick.
     * Runs continuously until interrupted.
     */
    public static Command manualControl(Turret turret, DoubleSupplier powerSupplier) {
        return new LambdaCommand()
                .named("TurretManual")
                .requires(turret)
                .setStart(() -> {})
                .setUpdate(() -> {
                    double power = powerSupplier.getAsDouble();
                    // Apply deadband
                    if (Math.abs(power) < 0.1) {
                        power = 0;
                    }
                    turret.spin(power);
                })
                .setStop(interrupted -> turret.stop())
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    /**
     * Stop the turret immediately
     */
    public static Command stop(Turret turret) {
        return new InstantCommand("TurretStop", turret::stop);
    }

    // ===== POSITION CONTROL =====

    /**
     * Move turret to a specific angle (degrees from center).
     * Completes when turret reaches target position.
     */
    public static Command goToAngle(Turret turret, double degrees) {
        return new LambdaCommand()
                .named("TurretGoTo")
                .requires(turret)
                .setStart(() -> turret.goToAngle(degrees))
                .setUpdate(() -> {})  // PID runs in subsystem periodic()
                .setStop(interrupted -> {
                    if (interrupted) turret.stop();
                })
                .setIsDone(() -> turret.atPosition())
                .setInterruptible(true);
    }

    /**
     * Move turret to center position (0 degrees)
     */
    public static Command goToCenter(Turret turret) {
        return goToAngle(turret, TurretConstants.POSITION_CENTER);
    }

    /**
     * Move turret to left 45 degrees
     */
    public static Command goToLeft45(Turret turret) {
        return goToAngle(turret, TurretConstants.POSITION_LEFT_45);
    }

    /**
     * Move turret to right 45 degrees
     */
    public static Command goToRight45(Turret turret) {
        return goToAngle(turret, TurretConstants.POSITION_RIGHT_45);
    }

    /**
     * Move turret to left 90 degrees
     */
    public static Command goToLeft90(Turret turret) {
        return goToAngle(turret, TurretConstants.POSITION_LEFT_90);
    }

    /**
     * Move turret to right 90 degrees
     */
    public static Command goToRight90(Turret turret) {
        return goToAngle(turret, TurretConstants.POSITION_RIGHT_90);
    }

    /**
     * Reset turret encoder (home position)
     */
    public static Command home(Turret turret) {
        return new InstantCommand("TurretHome", turret::home);
    }

    // ===== AUTO-ALIGN (VISION-BASED) =====

    /**
     * Auto-align turret to AprilTag using Limelight vision.
     * Runs continuously until interrupted (for TeleOp button hold).
     * Uses the chassis's Limelight for vision data.
     */
    public static Command autoAlign(Turret turret, SuperChassis chassis) {
        return new LambdaCommand()
                .named("TurretAutoAlign")
                .requires(turret)
                .setStart(() -> {
                    turret.startAutoAlign();
                })
                .setUpdate(() -> {
                    // Check if Limelight is connected and has valid data
                    if (chassis.isLLConnected()) {
                        // Get the last detected AprilTag ID
                        int detectedId = chassis.getLastDetectedId();

                        // Only align to alignment tags (20 and 24)
                        if (VisionConstants.isAlignmentTag(detectedId)) {
                            // Get horizontal offset (tx) from Limelight
                            double tx = chassis.getLLTx();
                            turret.setAlignmentError(tx);
                        } else {
                            // No valid alignment tag - stop turret
                            turret.setAlignmentError(0);
                        }
                    } else {
                        // Limelight not connected
                        turret.setAlignmentError(0);
                    }
                })
                .setStop(interrupted -> {
                    turret.stopAutoAlign();
                })
                .setIsDone(() -> false)  // Run until interrupted
                .setInterruptible(true);
    }

    /**
     * Auto-align turret to any visible AprilTag (not just alignment tags).
     * Useful for testing or special scenarios.
     */
    public static Command autoAlignAnyTag(Turret turret, SuperChassis chassis) {
        return new LambdaCommand()
                .named("TurretAutoAlignAny")
                .requires(turret)
                .setStart(() -> {
                    turret.startAutoAlign();
                })
                .setUpdate(() -> {
                    if (chassis.isLLConnected() && chassis.getLastDetectedId() != -1) {
                        double tx = chassis.getLLTx();
                        turret.setAlignmentError(tx);
                    } else {
                        turret.setAlignmentError(0);
                    }
                })
                .setStop(interrupted -> {
                    turret.stopAutoAlign();
                })
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    /**
     * Auto-align with joystick fallback.
     * If no valid target, allow manual control.
     */
    public static Command autoAlignWithFallback(Turret turret, SuperChassis chassis,
                                                  DoubleSupplier manualPower) {
        return new LambdaCommand()
                .named("TurretAutoAlignFallback")
                .requires(turret)
                .setStart(() -> {})
                .setUpdate(() -> {
                    boolean hasTarget = chassis.isLLConnected() &&
                            VisionConstants.isAlignmentTag(chassis.getLastDetectedId());

                    if (hasTarget) {
                        // Vision alignment mode
                        if (!turret.isAutoAligning()) {
                            turret.startAutoAlign();
                        }
                        double tx = chassis.getLLTx();
                        turret.setAlignmentError(tx);
                    } else {
                        // Manual fallback
                        if (turret.isAutoAligning()) {
                            turret.stopAutoAlign();
                        }
                        double power = manualPower.getAsDouble();
                        if (Math.abs(power) < 0.1) {
                            power = 0;
                        }
                        turret.spin(power);
                    }
                })
                .setStop(interrupted -> {
                    turret.stopAutoAlign();
                    turret.stop();
                })
                .setIsDone(() -> false)
                .setInterruptible(true);
    }

    /**
     * Align turret and hold when aligned.
     * Completes when turret is aligned to target.
     */
    public static Command alignAndHold(Turret turret, SuperChassis chassis) {
        return new LambdaCommand()
                .named("TurretAlignAndHold")
                .requires(turret)
                .setStart(() -> {
                    turret.startAutoAlign();
                })
                .setUpdate(() -> {
                    if (chassis.isLLConnected() &&
                        VisionConstants.isAlignmentTag(chassis.getLastDetectedId())) {
                        double tx = chassis.getLLTx();
                        turret.setAlignmentError(tx);
                    } else {
                        turret.setAlignmentError(0);
                    }
                })
                .setStop(interrupted -> {
                    // Keep turret at current position when done
                    if (!interrupted) {
                        turret.goToAngle(turret.getCurrentAngle());
                    } else {
                        turret.stopAutoAlign();
                    }
                })
                .setIsDone(() -> turret.isAligned())
                .setInterruptible(true);
    }
}
