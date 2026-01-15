package org.firstinspires.ftc.teamcode.Robot.Subsystems.Hood;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

/**
 * Hood Subsystem
 *
 * Controls a 300-degree rotation servo to adjust shot trajectory.
 * Used for distance-based aiming instead of RPM adjustment.
 *
 * The hood angle determines the ball's launch angle:
 * - Lower angles (15-30°): Flat trajectory for close shots
 * - Medium angles (40-50°): Standard trajectory for mid-range
 * - Higher angles (60-75°): Steep trajectory for far shots
 */
public class Hood implements Subsystem {

    public static final Hood INSTANCE = new Hood();

    // Hardware
    private Servo hoodServo;

    // State
    private double currentAngleDegrees = HoodConstants.DEFAULT_ANGLE_DEG;
    private double targetAngleDegrees = HoodConstants.DEFAULT_ANGLE_DEG;

    // Default command
    private Command defaultCommand = new NullCommand();

    @Override
    public void initialize() {
        try {
            hoodServo = ActiveOpMode.hardwareMap().get(Servo.class, HoodConstants.HOOD_SERVO_NAME);
            if (HoodConstants.SERVO_REVERSED) {
                hoodServo.setDirection(Servo.Direction.REVERSE);
            }
            // Set to default position
            setAngleDegrees(HoodConstants.DEFAULT_ANGLE_DEG);
        } catch (Exception e) {
            hoodServo = null;
            ActiveOpMode.telemetry().addData("Hood Error", "Servo not found: " + e.getMessage());
        }
    }

    @NonNull
    @Override
    public Command getDefaultCommand() {
        return defaultCommand;
    }

    public void setDefaultCommand(Command command) {
        this.defaultCommand = command;
    }

    @Override
    public void periodic() {
        if (HoodConstants.ENABLE_TELEMETRY) {
            updateTelemetry();
        }
    }

    // ===== ANGLE CONTROL =====

    /**
     * Set hood to a specific angle in degrees.
     * Angle is clamped to valid shooting range.
     *
     * @param degrees Target angle (15-75 degrees typical)
     */
    public void setAngleDegrees(double degrees) {
        if (hoodServo == null) return;

        // Clamp to valid range
        degrees = HoodConstants.clampAngle(degrees);
        targetAngleDegrees = degrees;
        currentAngleDegrees = degrees;

        // Convert to servo position
        double servoPosition = HoodConstants.degreesToServoPosition(degrees);
        hoodServo.setPosition(servoPosition);
    }

    /**
     * Set hood angle based on distance to target.
     * Uses projectile motion calculations for optimal trajectory.
     *
     * @param distanceInches Distance to target in inches
     */
    public void setHoodForDistance(double distanceInches) {
        double angle = HoodConstants.calculateAngleForDistance(distanceInches);
        setAngleDegrees(angle);
    }

    /**
     * Adjust hood angle by a delta amount.
     *
     * @param deltaDegrees Amount to adjust (positive = steeper, negative = flatter)
     */
    public void adjustAngle(double deltaDegrees) {
        setAngleDegrees(currentAngleDegrees + deltaDegrees);
    }

    // ===== PRESET POSITIONS =====

    /**
     * Set hood for close-range shots (flat trajectory)
     */
    public void setClose() {
        setAngleDegrees(HoodConstants.ANGLE_PRESET_CLOSE);
    }

    /**
     * Set hood for mid-range shots
     */
    public void setMid() {
        setAngleDegrees(HoodConstants.ANGLE_PRESET_MID);
    }

    /**
     * Set hood for far-range shots (steep trajectory)
     */
    public void setFar() {
        setAngleDegrees(HoodConstants.ANGLE_PRESET_FAR);
    }

    /**
     * Set hood to default position
     */
    public void setDefault() {
        setAngleDegrees(HoodConstants.DEFAULT_ANGLE_DEG);
    }

    // ===== STATE QUERIES =====

    /**
     * Get current hood angle in degrees
     */
    public double getAngleDegrees() {
        return currentAngleDegrees;
    }

    /**
     * Get target hood angle in degrees
     */
    public double getTargetAngleDegrees() {
        return targetAngleDegrees;
    }

    /**
     * Get current servo position (0.0 to 1.0)
     */
    public double getServoPosition() {
        return HoodConstants.degreesToServoPosition(currentAngleDegrees);
    }

    /**
     * Check if hood servo is initialized
     */
    public boolean isInitialized() {
        return hoodServo != null;
    }

    // ===== TELEMETRY =====

    private void updateTelemetry() {
        try {
            ActiveOpMode.telemetry().addData("--- HOOD ---", "");
            ActiveOpMode.telemetry().addData("Angle", "%.1f deg", currentAngleDegrees);
            ActiveOpMode.telemetry().addData("Servo Pos", "%.3f", getServoPosition());
            ActiveOpMode.telemetry().addData("Status", hoodServo != null ? "OK" : "NOT FOUND");
        } catch (Exception e) {
            // Telemetry not ready
        }
    }

    // ===== COMPONENT REGISTRATION =====

    public SubsystemComponent asCOMPONENT() {
        return new SubsystemComponent(INSTANCE);
    }
}
