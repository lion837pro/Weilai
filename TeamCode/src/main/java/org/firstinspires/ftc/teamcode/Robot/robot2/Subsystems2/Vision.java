package org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Lib.STZLite.Geometry.Pose;

import java.util.ArrayList;
import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class Vision implements Subsystem {

    public static final Vision INSTANCE = new Vision();

    private Limelight3A limelight;

    private double tx, ty, ta;
    private int lastDetectedId = -1;
    private List<LLResultTypes.FiducialResult> currentFiducials = new ArrayList<>();

    private Command defaultCommand = new NullCommand();

    @Override
    public void initialize() {
        try {
            limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, VisionConstants.limelightName);
            limelight.setPollRateHz(100);
            limelight.start();
        } catch (Exception e) {
            ActiveOpMode.telemetry().addData("Limelight Error", e.getMessage());
        }
    }

    @Override
    public void periodic() {
        if (limelight == null || !limelight.isConnected()) {
            clearFiducials();
            return;
        }

        try {
            LLResult result = limelight.getLatestResult();

            if (!validLLResult(result)) {
                clearFiducials();
                return;
            }

            long staleness = result.getStaleness();

            if (staleness < VisionConstants.STALENESS_THRESHOLD_MS) {
                // Fresh data
                this.tx = result.getTx();
                this.ty = result.getTy();
                this.ta = result.getTa();

                this.currentFiducials = result.getFiducialResults();

                if (this.currentFiducials != null && !this.currentFiducials.isEmpty()) {
                    this.lastDetectedId = this.currentFiducials.get(0).getFiducialId();
                } else {
                    this.lastDetectedId = -1;
                }
            } else {
                // Stale data
                clearFiducials();
            }
        } catch (Exception e) {
            clearFiducials();
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

    public SubsystemComponent asCOMPONENT() {
        return new SubsystemComponent(INSTANCE);
    }

    // ========== HELPER METHODS ==========

    private void clearFiducials() {
        if (this.currentFiducials != null) {
            this.currentFiducials.clear();
        }
        this.lastDetectedId = -1;
    }

    private boolean validLLResult(LLResult result) {
        return result != null && result.isValid();
    }

    // ========== GETTERS ==========

    public boolean isConnected() {
        return limelight != null && limelight.isConnected();
    }

    public boolean hasTarget() {
        return isConnected() && limelight.getLatestResult().isValid();
    }

    public boolean hasID(int id) {
        if (this.currentFiducials == null || this.currentFiducials.isEmpty()) {
            return false;
        }

        for (LLResultTypes.FiducialResult fiducial : this.currentFiducials) {
            if (fiducial.getFiducialId() == id) {
                return true;
            }
        }

        return false;
    }

    public double getTx() { return tx; }
    public double getTy() { return ty; }
    public double getTa() { return ta; }

    public int getLastDetectedId() { return lastDetectedId; }

    public List<LLResultTypes.FiducialResult> getCurrentFiducials() {
        return currentFiducials;
    }

    public Limelight3A getLimelight() {
        return limelight;
    }

    // ========== CONTROL METHODS ==========

    public void switchPipeline(int pipeline) {
        if (limelight != null) {
            limelight.pipelineSwitch(pipeline);
        }
    }

    /**
     * Calculate distance to target using trigonometry
     * @return distance in inches
     */
    public double getDistanceToTag() {
        if (!isConnected()) return 0.0;

        double ty = getTy();
        double h_target = VisionConstants.TARGET_HEIGHT;
        double h_cam = VisionConstants.CAMERA_HEIGHT;
        double angle_mount = VisionConstants.CAMERA_ANGLE;

        double angle_total_rad = Math.toRadians(angle_mount + ty);

        // Prevent division by zero
        if (Math.abs(Math.tan(angle_total_rad)) < 0.001) return 0.0;

        double distance = (h_target - h_cam) / Math.tan(angle_total_rad);

        return distance;
    }
}