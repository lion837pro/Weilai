package org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;

/**
 * Constants for the Turret subsystem.
 *
 * The turret rotates the shooter mechanism to aim at targets.
 * Uses a motor with 19.2:1 gearbox driving a 4.1:1 turret gear.
 * Total ratio: 19.2 * 4.1 = 78.72:1
 *
 * IMPORTANT: No limit switch - must manually zero turret at startup!
 * Position the turret pointing forward (center) before enabling.
 */
public class TurretConstants {

    // ===== HARDWARE NAMES =====
    public static final String TURRET_MOTOR_NAME = "turret";

    // ===== MOTOR CONFIGURATION =====
    public static final boolean MOTOR_INVERTED = false;
    public static final double TICKS_PER_REV = 28.0;        // GoBilda Yellow Jacket encoder ticks per motor revolution
    public static final double MOTOR_GEARBOX_RATIO = 19.2;  // Motor internal gearbox ratio
    public static final double TURRET_GEAR_RATIO = 4.1;     // External gear ratio (motor gear to turret gear)
    public static final double TOTAL_GEAR_RATIO = MOTOR_GEARBOX_RATIO * TURRET_GEAR_RATIO;  // 78.72:1

    // Calculated: Total ticks for one full turret rotation
    public static final double TICKS_PER_TURRET_REV = TICKS_PER_REV * TOTAL_GEAR_RATIO;  // 2204.16 ticks
    public static final double TICKS_PER_DEGREE = TICKS_PER_TURRET_REV / 360.0;          // ~6.12 ticks/degree

    // ===== TURRET LIMITS =====
    // Soft limits to prevent over-rotation and cable damage (degrees from center)
    // IMPORTANT: These are SOFTWARE limits only - no hardware limit switch!
    public static final double MAX_ANGLE_DEGREES = 90.0;   // Maximum rotation clockwise from center
    public static final double MIN_ANGLE_DEGREES = -90.0;  // Maximum rotation counter-clockwise from center

    // Corresponding tick limits
    public static final double MAX_TICKS = MAX_ANGLE_DEGREES * TICKS_PER_DEGREE;
    public static final double MIN_TICKS = MIN_ANGLE_DEGREES * TICKS_PER_DEGREE;

    // ===== CONTROL GAINS =====
    // Position PID for turret control (tuned for 78.72:1 ratio)
    public static final double kP = 0.015;     // Proportional gain
    public static final double kI = 0.0;       // Integral gain (disabled to prevent oscillation)
    public static final double kD = 0.003;     // Derivative gain for damping
    public static final double kS = 0.04;      // Static friction compensation

    // ===== AUTO-ALIGN CONFIGURATION =====
    // PID for vision-based alignment (uses Limelight tx)
    public static final double ALIGN_kP = 0.012;    // Proportional for alignment
    public static final double ALIGN_kD = 0.002;    // Derivative for alignment
    public static final double ALIGN_DEADBAND = 1.5; // Degrees - target is "aligned" if within this

    // ===== MOTION CONSTRAINTS =====
    public static final double MAX_POWER = 0.6;           // Maximum motor power (reduced for safety)
    public static final double MANUAL_POWER_SCALE = 0.4;  // Scale for manual joystick control
    public static final double POSITION_TOLERANCE = 8.0;  // Ticks tolerance for "at position"

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
