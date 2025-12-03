package org.firstinspires.ftc.teamcode.Robot.Subsystems.Spindexer;

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
    public static final String COLOR_SENSOR_1_NAME = "colorSensor1";  // Detects ball at position 0/1
    public static final String COLOR_SENSOR_2_NAME = "colorSensor2";  // Detects ball at position 2/3

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

    // ===== TIMING =====
    public static final double HOMING_TIMEOUT_MS = 3000;     // Max time to search for home
    public static final double INDEX_TIMEOUT_MS = 1000;      // Max time to move to position
    public static final double SETTLE_TIME_MS = 50;          // Time to wait after reaching position
    public static final double BALL_DETECT_DEBOUNCE_MS = 50; // Debounce for color sensor readings

    // ===== COLOR SENSOR THRESHOLDS =====
    // These detect if a ball is present in a slot
    // Tune these based on your specific balls and sensors
    public static final int COLOR_PROXIMITY_THRESHOLD = 150;  // Proximity value indicating ball present

    // Ball color detection (for sorting if needed)
    public static final int RED_THRESHOLD = 200;   // Red channel value for red ball
    public static final int BLUE_THRESHOLD = 200;  // Blue channel value for blue ball

    // ===== DIRECTION OPTIMIZATION =====
    // When indexing, choose shortest rotation direction
    public static final boolean OPTIMIZE_ROTATION_DIRECTION = true;

    // ===== DEBUG =====
    public static final boolean ENABLE_TELEMETRY = true;
}
