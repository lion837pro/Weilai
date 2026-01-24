package org.firstinspires.ftc.teamcode.Robot.Subsystems.Hood;

/*
 * HOOD CONSTANTS - COMMENTED OUT
 * Using RPM-based distance shooting instead of hood angle adjustment.
 * To re-enable, uncomment this entire file.
 */

/*
public class HoodConstants {

    public static final String HOOD_SERVO_NAME = "hoodServo";
    public static final boolean SERVO_REVERSED = false;
    public static final double SERVO_RANGE_DEGREES = 300.0;

    public static final double MIN_SHOOTING_ANGLE_DEG = 15.0;
    public static final double MAX_SHOOTING_ANGLE_DEG = 75.0;
    public static final double DEFAULT_ANGLE_DEG = 45.0;

    public static final double FIXED_SHOOTING_RPM = 2500.0;

    public static final double CLOSE_DISTANCE_INCHES = 24.0;
    public static final double FAR_DISTANCE_INCHES = 72.0;

    public static final double CLOSE_SHOT_ANGLE_DEG = 20.0;
    public static final double MID_SHOT_ANGLE_DEG = 45.0;
    public static final double FAR_SHOT_ANGLE_DEG = 65.0;

    public static final double ANGLE_PRESET_CLOSE = 20.0;
    public static final double ANGLE_PRESET_MID = 45.0;
    public static final double ANGLE_PRESET_FAR = 65.0;

    public static double degreesToServoPosition(double degrees) {
        degrees = Math.max(0, Math.min(SERVO_RANGE_DEGREES, degrees));
        return degrees / SERVO_RANGE_DEGREES;
    }

    public static double servoPositionToDegrees(double position) {
        position = Math.max(0, Math.min(1.0, position));
        return position * SERVO_RANGE_DEGREES;
    }

    public static double calculateAngleForDistance(double distanceInches) {
        if (distanceInches <= CLOSE_DISTANCE_INCHES) return CLOSE_SHOT_ANGLE_DEG;
        if (distanceInches >= FAR_DISTANCE_INCHES) return FAR_SHOT_ANGLE_DEG;
        double t = (distanceInches - CLOSE_DISTANCE_INCHES) / (FAR_DISTANCE_INCHES - CLOSE_DISTANCE_INCHES);
        return CLOSE_SHOT_ANGLE_DEG + t * (FAR_SHOT_ANGLE_DEG - CLOSE_SHOT_ANGLE_DEG);
    }

    public static double clampAngle(double degrees) {
        return Math.max(MIN_SHOOTING_ANGLE_DEG, Math.min(MAX_SHOOTING_ANGLE_DEG, degrees));
    }

    public static final boolean ENABLE_TELEMETRY = true;
}
*/
