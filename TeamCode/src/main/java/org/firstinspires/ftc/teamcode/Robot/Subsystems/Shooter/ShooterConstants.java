package org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.VisionConstants;

public class ShooterConstants {
    public static final String shootername1 = "Sh1";
    //public static final String shootername2 = "Sh2";

    public static final double TICKS_PER_REV = 28.0; // For GoBilda Yellow Jacket 1:1
    public static final double MAX_RPM = 6000.0;

    public static double rpmToTicksPerSecond(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }

    public static double ticksPerSecondToRPM(double tps) {
        return (tps * 60.0) / TICKS_PER_REV;
    }

    public static final boolean shootername2inverted = false;
    public static final boolean shootername1inverted = true;


    // ===== ACCELERATION LIMITING (PREVENTS BELT SLIP) =====
    /**
     * Maximum rate of power change per second
     * <p>
     * Examples:
     * - 1.0 = Takes 1 second to go from 0% to 100% power (very gentle)
     * - 1.5 = Takes 0.67 seconds (gentle, good for belt slip prevention)
     * - 2.0 = Takes 0.5 seconds (moderate)
     * - 3.0 = Takes 0.33 seconds (aggressive)
     * - 5.0 = Takes 0.2 seconds (very aggressive)
     * - 8.0 = Takes 0.125 seconds (maximum responsiveness)
     * <p>
     * Now that belt slip is fixed, increased to 8.0 for fast response
     */
    public static final double MAX_ACCELERATION = 8.0;

    // ===== TUNING MODE SELECTOR =====
    // Set this to true if you want velocity-dependent kV (advanced)
    // Set to false for simple single-kV control (recommended to start)
    public static final boolean USE_ADAPTIVE_KV = false;

    // ===== SIMPLE MODE CONSTANTS (USE_ADAPTIVE_KV = false) =====
    // Reduced kP to minimize oscillation now that velocity reading is fixed
    public static final double kP = 0.0012;
    public static final double kS = 0.10;
    public static final double kV = 0.00040;

    // ===== ADVANCED MODE CONSTANTS (USE_ADAPTIVE_KV = true) =====
    // Use these if simple mode doesn't work well across your velocity range
    public static final double LOW_SPEED_KV = 0.00050;   // For velocities < 1500 tps (close shots)
    public static final double HIGH_SPEED_KV = 0.00035;  // For velocities > 2500 tps (far shots)
    public static final double TRANSITION_SPEED = 2000;   // Where to start blending

    // ===== TOLERANCE =====
    public static final double velocityTolerance = 200;

    // ===== VELOCITY RANGES FOR AUTO-AIM =====
    // These help you understand what velocities correspond to what distances
    public static final double MIN_SHOOTING_RPM = 1400;  // Minimum safe RPM
    public static final double MAX_SHOOTING_RPM = 4500;  // Maximum safe RPM

    /**
     * Converts distance to expected RPM using your auto-aim formula
     * Useful for testing and visualization
     */

    // From VisionConstants: RPM = BASE_RPM + (distance * RPM_PER_INCH)
    // You can import VisionConstants or duplicate the formula here
    public static double distanceToRPM(double distanceInches) {
        return VisionConstants.BASE_RPM + (distanceInches * VisionConstants.RPM_PER_INCH);
    }
    /**
     * Estimates distance from RPM (inverse of above)
     * Useful for debugging
     */
    public static double rpmToDistance(double rpm) {
        return (rpm - VisionConstants.BASE_RPM) / VisionConstants.RPM_PER_INCH;
    }
}


