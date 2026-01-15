package org.firstinspires.ftc.teamcode.Robot.Subsystems.Hood;

/**
 * Constants for the Hood subsystem.
 *
 * The hood uses a 300-degree rotation servo to adjust shot trajectory.
 * Instead of varying RPM based on distance, we keep RPM constant and
 * adjust the hood angle for different distances.
 */
public class HoodConstants {

    // ===== HARDWARE NAMES =====
    public static final String HOOD_SERVO_NAME = "hoodServo";

    // ===== SERVO CONFIGURATION =====
    public static final boolean SERVO_REVERSED = false;

    // 300-degree servo range
    public static final double SERVO_RANGE_DEGREES = 300.0;

    // ===== SHOOTING ANGLE LIMITS =====
    // Usable range within the 300-degree servo rotation
    public static final double MIN_SHOOTING_ANGLE_DEG = 15.0;   // Flattest trajectory (close shots)
    public static final double MAX_SHOOTING_ANGLE_DEG = 75.0;   // Steepest trajectory (far shots)
    public static final double DEFAULT_ANGLE_DEG = 45.0;        // Neutral position

    // ===== FIXED SHOOTER RPM =====
    // With hood-based distance control, we use a constant RPM
    public static final double FIXED_SHOOTING_RPM = 2000.0;

    // ===== DISTANCE-TO-ANGLE MAPPING =====
    // Distance range (inches)
    public static final double CLOSE_DISTANCE_INCHES = 24.0;
    public static final double FAR_DISTANCE_INCHES = 72.0;

    // Angle mapping for distances
    public static final double CLOSE_SHOT_ANGLE_DEG = 20.0;     // 24 inches
    public static final double MID_SHOT_ANGLE_DEG = 45.0;       // 48 inches
    public static final double FAR_SHOT_ANGLE_DEG = 65.0;       // 72 inches

    // ===== PRESET ANGLES =====
    public static final double ANGLE_PRESET_CLOSE = 20.0;
    public static final double ANGLE_PRESET_MID = 45.0;
    public static final double ANGLE_PRESET_FAR = 65.0;

    // ===== CONVERSION METHODS =====

    /**
     * Convert degrees to servo position (0.0 to 1.0)
     * 0 degrees = servo position 0.0
     * 300 degrees = servo position 1.0
     */
    public static double degreesToServoPosition(double degrees) {
        // Clamp to valid range
        degrees = Math.max(0, Math.min(SERVO_RANGE_DEGREES, degrees));
        return degrees / SERVO_RANGE_DEGREES;
    }

    /**
     * Convert servo position to degrees
     */
    public static double servoPositionToDegrees(double position) {
        position = Math.max(0, Math.min(1.0, position));
        return position * SERVO_RANGE_DEGREES;
    }

    /**
     * Calculate optimal hood angle for a given distance.
     * Uses linear interpolation between close and far angles.
     *
     * @param distanceInches Distance to target in inches
     * @return Hood angle in degrees
     */
    public static double calculateAngleForDistance(double distanceInches) {
        // Clamp distance to valid range
        if (distanceInches <= CLOSE_DISTANCE_INCHES) {
            return CLOSE_SHOT_ANGLE_DEG;
        }
        if (distanceInches >= FAR_DISTANCE_INCHES) {
            return FAR_SHOT_ANGLE_DEG;
        }

        // Linear interpolation
        double t = (distanceInches - CLOSE_DISTANCE_INCHES) /
                   (FAR_DISTANCE_INCHES - CLOSE_DISTANCE_INCHES);
        return CLOSE_SHOT_ANGLE_DEG + t * (FAR_SHOT_ANGLE_DEG - CLOSE_SHOT_ANGLE_DEG);
    }

    /**
     * Clamp angle to valid shooting range
     */
    public static double clampAngle(double degrees) {
        return Math.max(MIN_SHOOTING_ANGLE_DEG, Math.min(MAX_SHOOTING_ANGLE_DEG, degrees));
    }

    // ===== DEBUG =====
    public static final boolean ENABLE_TELEMETRY = true;
}
