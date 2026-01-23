package org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.VisionConstants;

/**
 * Constants for the Spindexer (Spinning Indexer) subsystem.
 *
 * The spindexer can hold 3 balls and has 6 preset positions:
 * - Positions 0, 2, 4 are aligned with the INTAKE (ball loading positions)
 * - Positions 1, 3, 5 are aligned with the SHOOTER (ball firing positions)
 *
 * HARDWARE: 312 RPM motor with encoder for position control
 *
 * Physical layout (top view):
 *           SHOOTER
 *              |
 *         1 ---+--- 0
 *        /           \
 *       2      *      5  <-- Center axle
 *        \           /
 *         3 ---+--- 4
 *              |
 *           INTAKE
 */
public class SpindexerConstants {

    // ===== HARDWARE NAMES =====
    public static final String MOTOR_NAME = "spin";  // Motor name for spindexer
    public static final String LIMIT_SWITCH_NAME = "spindexerLimit";  // Magnetic limit switch for homing
    public static final String COLOR_SENSOR_1_NAME = "colorS1";  // Detects ball at intake
    public static final String COLOR_SENSOR_2_NAME = "colorS2";  // Secondary sensor (optional)

    // ===== LIMIT SWITCH CONFIGURATION =====
    // Polarity: true = active-low (triggered when LOW), false = active-high (triggered when HIGH)
    public static final boolean LIMIT_SWITCH_ACTIVE_LOW = true;

    // ===== MOTOR CONFIGURATION =====
    public static final boolean MOTOR_INVERTED = false;  // Set to true to reverse motor direction

    // ===== MOTOR SPECS (312 RPM GoBilda Yellow Jacket) =====
    // 312 RPM motor has 537.7 PPR (pulses per revolution at output shaft)
    public static final double TICKS_PER_MOTOR_REV = 537.7;

    // Gear ratio from motor to spindexer (if any external gearing)
    public static final double GEAR_RATIO = 1.0;  // 1:1 direct drive, adjust if geared

    // Final ticks per spindexer revolution
    public static final double TICKS_PER_SPINDEXER_REV = TICKS_PER_MOTOR_REV * GEAR_RATIO;
    public static final double TICKS_PER_DEGREE = TICKS_PER_SPINDEXER_REV / 360.0;

    // ===== POSITION PRESETS =====
    public static final int POSITION_COUNT = 6;
    public static final int SLOTS_COUNT = 3;  // Number of ball slots

    // 6 positions at 60 degree intervals
    public static final double DEGREES_PER_POSITION = 60.0;
    public static final double TICKS_PER_POSITION = DEGREES_PER_POSITION * TICKS_PER_DEGREE;

    // ===== SHOOTING SEQUENCE TIMING =====
    public static final long FEED_DURATION_MS = 300;  // Time to feed ball into shooter (milliseconds)

    // Intake positions (where balls load INTO the spindexer)
    public static final int INTAKE_POSITION_1 = 0;  // Slot A at intake
    public static final int INTAKE_POSITION_2 = 2;  // Slot B at intake
    public static final int INTAKE_POSITION_3 = 4;  // Slot C at intake

    // Shooter positions (where balls feed TO the shooter)
    public static final int SHOOTER_POSITION_1 = 1;  // Slot A at shooter
    public static final int SHOOTER_POSITION_2 = 3;  // Slot B at shooter
    public static final int SHOOTER_POSITION_3 = 5;  // Slot C at shooter

    /**
     * Get encoder ticks for a specific position index (0-5)
     */
    public static double getPositionTicks(int positionIndex) {
        return positionIndex * TICKS_PER_POSITION;
    }

    /**
     * Get the intake position index for a given slot (0, 1, or 2)
     */
    public static int getIntakePosition(int slotIndex) {
        return slotIndex * 2;  // 0 -> 0, 1 -> 2, 2 -> 4
    }

    /**
     * Get the shooter position index for a given slot (0, 1, or 2)
     */
    public static int getShooterPosition(int slotIndex) {
        return (slotIndex * 2) + 1;  // 0 -> 1, 1 -> 3, 2 -> 5
    }

    // ===== CONTROL GAINS (Position PID) =====
    public static final double kP = 0.008;     // Proportional gain
    public static final double kI = 0.0;       // Integral gain (disabled to prevent oscillation)
    public static final double kD = 0.0004;    // Derivative gain for damping

    // Feedforward
    public static final double kS = 0.05;  // Static friction compensation

    // ===== MOTION CONSTRAINTS =====
    public static final double MAX_POWER = 0.7;           // Maximum motor power
    public static final double HOMING_POWER = 0.25;       // Slow power for homing routine
    public static final double POSITION_TOLERANCE = 15.0; // Encoder ticks tolerance for "at position"

    // ===== MECHANICAL OFFSET =====
    // Offset angle to move ball away from shooter wheel during spin-up
    // This allows the shooter to reach target RPM without ball friction
    public static final double SHOOTER_CLEARANCE_OFFSET_DEGREES = 60.0;
    public static final double SHOOTER_CLEARANCE_OFFSET_TICKS =
            SHOOTER_CLEARANCE_OFFSET_DEGREES * TICKS_PER_DEGREE;

    // ===== TIMING =====
    public static final double HOMING_TIMEOUT_MS = 3000;     // Max time to search for home
    public static final double INDEX_TIMEOUT_MS = 1000;      // Max time to move to position
    public static final double SETTLE_TIME_MS = 50;          // Time to wait after reaching position

    // ===== COLOR SENSOR THRESHOLDS =====
    public static final float COLOR_SENSOR_GAIN = 2.0f;
    public static final double COLOR_PROXIMITY_THRESHOLD = 45;  // Ball present if < this distance (mm)

    // ===== BALL COLOR DETECTION THRESHOLDS =====
    // GREEN ball detection thresholds
    public static final int GREEN_MIN_G = 100;
    public static final int GREEN_MAX_R = 150;
    public static final int GREEN_MAX_B = 150;
    public static final double GREEN_RATIO_THRESHOLD = 1.2;

    // PURPLE ball detection thresholds
    public static final int PURPLE_MIN_R = 80;
    public static final int PURPLE_MIN_B = 80;
    public static final int PURPLE_MAX_G = 120;
    public static final double PURPLE_RB_MIN_RATIO = 0.7;
    public static final double PURPLE_RB_MAX_RATIO = 1.4;

    /**
     * Determine ball color from RGB values.
     */
    public static VisionConstants.BallColor detectBallColor(int red, int green, int blue) {
        // Check for GREEN ball first
        if (green >= GREEN_MIN_G &&
            red <= GREEN_MAX_R &&
            blue <= GREEN_MAX_B &&
            green > red * GREEN_RATIO_THRESHOLD &&
            green > blue * GREEN_RATIO_THRESHOLD) {
            return VisionConstants.BallColor.GREEN;
        }

        // Check for PURPLE ball
        if (red >= PURPLE_MIN_R &&
            blue >= PURPLE_MIN_B &&
            green <= PURPLE_MAX_G) {
            double rbRatio = (double) red / Math.max(blue, 1);
            if (rbRatio >= PURPLE_RB_MIN_RATIO && rbRatio <= PURPLE_RB_MAX_RATIO) {
                return VisionConstants.BallColor.PURPLE;
            }
        }

        return VisionConstants.BallColor.UNKNOWN;
    }

    // ===== DIRECTION OPTIMIZATION =====
    public static final boolean OPTIMIZE_ROTATION_DIRECTION = true;

    // ===== DEBUG =====
    public static final boolean ENABLE_TELEMETRY = true;
}
