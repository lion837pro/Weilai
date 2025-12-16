package org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;

/**
 * Constants for the Turret subsystem.
 *
 * The turret rotates the shooter mechanism to aim at targets.
 * Uses a single motor with 8:1 gear ratio.
 */
public class TurretConstants {

    // ===== HARDWARE NAMES =====
    public static final String TURRET_MOTOR_NAME = "turret";

    // ===== MOTOR CONFIGURATION =====
    public static final boolean MOTOR_INVERTED = false;
    public static final double TICKS_PER_REV = 28.0;  // GoBilda Yellow Jacket encoder ticks per motor revolution
    public static final double GEAR_RATIO = 8.0;      // 8:1 gear ratio (motor:turret)

    // Calculated: Total ticks for one full turret rotation
    public static final double TICKS_PER_TURRET_REV = TICKS_PER_REV * GEAR_RATIO;
    public static final double TICKS_PER_DEGREE = TICKS_PER_TURRET_REV / 360.0;

    // ===== TURRET LIMITS =====
    // Soft limits to prevent over-rotation (degrees from center)
    public static final double MAX_ANGLE_DEGREES = 180.0;  // Maximum rotation from center
    public static final double MIN_ANGLE_DEGREES = -180.0; // Minimum rotation from center

    // Corresponding tick limits
    public static final double MAX_TICKS = MAX_ANGLE_DEGREES * TICKS_PER_DEGREE;
    public static final double MIN_TICKS = MIN_ANGLE_DEGREES * TICKS_PER_DEGREE;

    // ===== CONTROL GAINS =====
    // Position PID for turret control
    public static final double kP = 0.02;      // Proportional gain
    public static final double kI = 0.0;       // Integral gain (disabled to prevent oscillation)
    public static final double kD = 0.005;     // Derivative gain for damping
    public static final double kS = 0.03;      // Static friction compensation

    // ===== AUTO-ALIGN CONFIGURATION =====
    // PID for vision-based alignment (uses Limelight tx)
    public static final double ALIGN_kP = 0.015;    // Proportional for alignment
    public static final double ALIGN_kD = 0.002;    // Derivative for alignment
    public static final double ALIGN_DEADBAND = 1.5; // Degrees - target is "aligned" if within this

    // ===== MOTION CONSTRAINTS =====
    public static final double MAX_POWER = 0.7;           // Maximum motor power
    public static final double MANUAL_POWER_SCALE = 0.5;  // Scale for manual joystick control
    public static final double POSITION_TOLERANCE = 5.0;  // Ticks tolerance for "at position"

    // ===== PRESET POSITIONS =====
    // Common turret positions (degrees from center)
    public static final double POSITION_CENTER = 0.0;
    public static final double POSITION_LEFT_45 = -45.0;
    public static final double POSITION_RIGHT_45 = 45.0;
    public static final double POSITION_LEFT_90 = -90.0;
    public static final double POSITION_RIGHT_90 = 90.0;

    /**
     * Convert degrees to encoder ticks
     */
    public static double degreesToTicks(double degrees) {
        return degrees * TICKS_PER_DEGREE;
    }

    /**
     * Convert encoder ticks to degrees
     */
    public static double ticksToDegrees(double ticks) {
        return ticks / TICKS_PER_DEGREE;
    }

    /**
     * Clamp angle to valid range
     */
    public static double clampAngle(double degrees) {
        return Math.max(MIN_ANGLE_DEGREES, Math.min(MAX_ANGLE_DEGREES, degrees));
    }

    // ===== DEBUG =====
    public static final boolean ENABLE_TELEMETRY = true;
}
