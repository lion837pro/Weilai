package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.ArrayList;
import java.util.List;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class SimpleVision implements Subsystem {

    public static final SimpleVision INSTANCE = new SimpleVision();

    // Configuration
    private static final String LIMELIGHT_NAME = "limelight";
    private static final double STALENESS_THRESHOLD_MS = 250;

    // Camera mounting parameters (in inches)
    private static final double TARGET_HEIGHT = 13.5;  // AprilTag height from floor
    private static final double CAMERA_HEIGHT = 9.5;   // Limelight height from floor
    private static final double CAMERA_ANGLE = 30;     // Camera tilt angle (degrees)

    // Limelight hardware
    private Limelight3A limelight;

    // Target data
    private double tx = 0;  // Horizontal offset
    private double ty = 0;  // Vertical offset
    private double ta = 0;  // Target area
    private int currentTagId = -1;
    private boolean hasValidTarget = false;

    // Fiducial tracking
    private List<LLResultTypes.FiducialResult> detectedTags = new ArrayList<>();

    @Override
    public void initialize() {
        try {
            limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, LIMELIGHT_NAME);
            limelight.setPollRateHz(100);
            limelight.start();

            ActiveOpMode.telemetry().addData("Vision", "Limelight initialized successfully");
        } catch (Exception e) {
            ActiveOpMode.telemetry().addData("Vision Error", "Failed to initialize Limelight: " + e.getMessage());
        }
    }

    @Override
    public void periodic() {
        if (!isConnected()) {
            clearTargetData();
            return;
        }

        try {
            LLResult result = limelight.getLatestResult();

            if (result == null || !result.isValid()) {
                clearTargetData();
                return;
            }

            // Check data freshness
            long staleness = result.getStaleness();
            if (staleness > STALENESS_THRESHOLD_MS) {
                clearTargetData();
                return;
            }

            // Update target data
            this.tx = result.getTx();
            this.ty = result.getTy();
            this.ta = result.getTa();
            this.hasValidTarget = true;

            // Update fiducial data
            this.detectedTags = result.getFiducialResults();
            if (detectedTags != null && !detectedTags.isEmpty()) {
                this.currentTagId = detectedTags.get(0).getFiducialId();
            } else {
                this.currentTagId = -1;
            }

            // Update telemetry
            updateTelemetry();

        } catch (Exception e) {
            clearTargetData();
            ActiveOpMode.telemetry().addData("Vision Error", e.getMessage());
        }
    }

    /**
     * Clear all target data
     */
    private void clearTargetData() {
        this.tx = 0;
        this.ty = 0;
        this.ta = 0;
        this.hasValidTarget = false;
        this.currentTagId = -1;
        if (this.detectedTags != null) {
            this.detectedTags.clear();
        }
    }

    /**
     * Update telemetry with vision data
     */
    private void updateTelemetry() {
        if (hasValidTarget) {
            ActiveOpMode.telemetry().addData("Vision", "Target Acquired");
            ActiveOpMode.telemetry().addData("Tag ID", currentTagId);
            ActiveOpMode.telemetry().addData("TX (Horizontal)", "%.2f deg", tx);
            ActiveOpMode.telemetry().addData("TY (Vertical)", "%.2f deg", ty);
            ActiveOpMode.telemetry().addData("TA (Area)", "%.2f%%", ta);
            ActiveOpMode.telemetry().addData("Distance", "%.1f in", calculateDistance());
        } else {
            ActiveOpMode.telemetry().addData("Vision", "No Target");
        }
    }

    /**
     * Check if Limelight is connected
     */
    public boolean isConnected() {
        return limelight != null && limelight.isConnected();
    }

    /**
     * Check if we have a valid target
     */
    public boolean hasTarget() {
        return hasValidTarget;
    }

    /**
     * Get horizontal offset to target (degrees)
     */
    public double getTx() {
        return tx;
    }

    /**
     * Get vertical offset to target (degrees)
     */
    public double getTy() {
        return ty;
    }

    /**
     * Get target area (percentage of image)
     */
    public double getTa() {
        return ta;
    }

    /**
     * Get current detected tag ID
     */
    public int getCurrentTagId() {
        return currentTagId;
    }

    /**
     * Check if a specific tag ID is detected
     */
    public boolean hasTagId(int id) {
        if (detectedTags == null || detectedTags.isEmpty()) {
            return false;
        }

        for (LLResultTypes.FiducialResult tag : detectedTags) {
            if (tag.getFiducialId() == id) {
                return true;
            }
        }
        return false;
    }

    /**
     * Get all detected tags
     */
    public List<LLResultTypes.FiducialResult> getDetectedTags() {
        return new ArrayList<>(detectedTags);
    }

    /**
     * Calculate distance to target using trigonometry
     * @return distance in inches
     */
    public double calculateDistance() {
        if (!hasValidTarget) return 0.0;

        // Total angle from horizontal
        double totalAngleRad = Math.toRadians(CAMERA_ANGLE + ty);

        // Prevent division by zero
        if (Math.abs(Math.tan(totalAngleRad)) < 0.001) {
            return 0.0;
        }

        // Calculate distance using height difference and angle
        double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(totalAngleRad);

        // Sanity check
        if (distance < 0 || distance > 200) {
            return 0.0;
        }

        return distance;
    }

    /**
     * Switch Limelight pipeline
     * @param pipeline Pipeline number to switch to
     */
    public void switchPipeline(int pipeline) {
        if (isConnected()) {
            limelight.pipelineSwitch(pipeline);
        }
    }

    /**
     * Update robot orientation for Limelight MegaTag2
     * @param robotYaw Robot heading in radians
     */
    public void updateRobotOrientation(double robotYaw) {
        if (isConnected()) {
            limelight.updateRobotOrientation(robotYaw);
        }
    }

    /**
     * Get alignment error for centering on target
     * @return Error in degrees (negative = turn left, positive = turn right)
     */
    public double getAlignmentError() {
        return hasValidTarget ? -tx : 0.0;
    }

    /**
     * Check if we're aligned to target (within tolerance)
     * @param tolerance Tolerance in degrees
     */
    public boolean isAligned(double tolerance) {
        return hasValidTarget && Math.abs(tx) < tolerance;
    }

    /**
     * Get recommended shooter RPM based on distance
     * This is a simple linear model - adjust based on your shooter
     */
    public double getRecommendedRPM() {
        if (!hasValidTarget) return 1400;  // Default RPM

        double distance = calculateDistance();

        // Simple linear model: BASE_RPM + (distance * RPM_PER_INCH)
        double rpm = 1400 + (distance * 15);

        // Clamp to safe range
        return Math.max(1400, Math.min(4500, rpm));
    }

    public SubsystemComponent asCOMPONENT() {
        return new SubsystemComponent(INSTANCE);
    }
}