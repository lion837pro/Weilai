package org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.VisionConstants;

/**
 * Constants for the Spindexer (Spinning Indexer) subsystem.
 *
 * The spindexer can hold 3 balls and has 6 preset positions:
 * - Positions 0, 2, 4 are aligned with the INTAKE (ball loading positions)
 * - Positions 1, 3, 5 are aligned with the SHOOTER (ball firing positions)
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
    public static final String MOTOR_NAME = "spindexer";
    public static final String LIMIT_SWITCH_NAME = "spindexerLimit";  // Magnetic limit switch for homing
    public static final String COLOR_SENSOR_1_NAME = "colorSensor1";  // Detects ball at intake
    public static final String COLOR_SENSOR_2_NAME = "colorSensor2";  // Secondary sensor (optional)

    // ===== MOTOR CONFIGURATION =====
    public static final boolean MOTOR_INVERTED = false;
    public static final double TICKS_PER_REV = 28.0;  // GoBilda Yellow Jacket encoder ticks per motor revolution
    public static final double GEAR_RATIO = 20.0;     // External gear ratio (motor:spindexer)

    // Calculated: Total ticks for one full spindexer rotation
    public static final double TICKS_PER_SPINDEXER_REV = TICKS_PER_REV * GEAR_RATIO;

    // ===== POSITION PRESETS (in encoder ticks from home) =====
    // 6 positions at 60 degree intervals
    public static final double DEGREES_PER_POSITION = 60.0;
    public static final double TICKS_PER_DEGREE = TICKS_PER_SPINDEXER_REV / 360.0;

    // Position indices
    public static final int POSITION_COUNT = 6;
    public static final int SLOTS_COUNT = 3;  // Number of ball slots

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
        return positionIndex * DEGREES_PER_POSITION * TICKS_PER_DEGREE;
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

    // ===== CONTROL GAINS =====
    // Position PID for precise indexing
    public static final double kP = 0.01;
    public static final double kI = 0.0;
    public static final double kD = 0.001;

    // Feedforward for consistent motion
    public static final double kS = 0.05;  // Static friction compensation
    public static final double kV = 0.0;   // Velocity feedforward (not needed for position control)

    // ===== MOTION CONSTRAINTS =====
    public static final double MAX_POWER = 0.8;           // Maximum motor power
    public static final double HOMING_POWER = 0.3;        // Slow power for homing routine
    public static final double POSITION_TOLERANCE = 5.0;  // Ticks tolerance for "at position"
    public static final double VELOCITY_TOLERANCE = 10.0; // Ticks/sec tolerance for "stopped"

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
    public static final double BALL_DETECT_DEBOUNCE_MS = 50; // Debounce for color sensor readings

    // ===== COLOR SENSOR THRESHOLDS =====
    // Proximity threshold for ball detection (MM)
    public static final double COLOR_PROXIMITY_THRESHOLD = 150;  // Ball present if < this distance

    // ===== BALL COLOR DETECTION THRESHOLDS =====
    // These values need tuning based on your specific color sensor and ball colors
    // The color sensor returns RGB values (0-255 range typically)

    // GREEN ball detection thresholds
    // Green balls typically have high green channel and lower red/blue
    public static final int GREEN_MIN_G = 100;  // Minimum green channel value
    public static final int GREEN_MAX_R = 150;  // Maximum red channel for green ball
    public static final int GREEN_MAX_B = 150;  // Maximum blue channel for green ball
    public static final double GREEN_RATIO_THRESHOLD = 1.2;  // G must be this much higher than R and B

    // PURPLE ball detection thresholds
    // Purple balls have high red and blue, lower green
    public static final int PURPLE_MIN_R = 80;   // Minimum red channel value
    public static final int PURPLE_MIN_B = 80;   // Minimum blue channel value
    public static final int PURPLE_MAX_G = 120;  // Maximum green channel for purple ball
    public static final double PURPLE_RB_MIN_RATIO = 0.7;  // R/B ratio should be close (0.7-1.4)
    public static final double PURPLE_RB_MAX_RATIO = 1.4;

    /**
     * Determine ball color from RGB values.
     * Uses VisionConstants.BallColor enum.
     */
    public static VisionConstants.BallColor detectBallColor(int red, int green, int blue) {
        // Check for GREEN ball first
        // Green has high green channel, and green is significantly higher than red and blue
        if (green >= GREEN_MIN_G &&
            red <= GREEN_MAX_R &&
            blue <= GREEN_MAX_B &&
            green > red * GREEN_RATIO_THRESHOLD &&
            green > blue * GREEN_RATIO_THRESHOLD) {
            return VisionConstants.BallColor.GREEN;
        }

        // Check for PURPLE ball
        // Purple has high red and blue, lower green
        if (red >= PURPLE_MIN_R &&
            blue >= PURPLE_MIN_B &&
            green <= PURPLE_MAX_G) {
            // Check that red and blue are close to each other (ratio check)
            double rbRatio = (double) red / Math.max(blue, 1);
            if (rbRatio >= PURPLE_RB_MIN_RATIO && rbRatio <= PURPLE_RB_MAX_RATIO) {
                return VisionConstants.BallColor.PURPLE;
            }
        }

        // Can't determine color
        return VisionConstants.BallColor.UNKNOWN;
    }

    // ===== DIRECTION OPTIMIZATION =====
    // When indexing, choose shortest rotation direction
    public static final boolean OPTIMIZE_ROTATION_DIRECTION = true;

    // ===== DEBUG =====
    public static final boolean ENABLE_TELEMETRY = true;
}
