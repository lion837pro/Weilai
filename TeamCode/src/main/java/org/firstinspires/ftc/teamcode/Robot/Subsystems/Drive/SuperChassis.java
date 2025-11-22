package org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive;



import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//import org.firstinspires.ftc.teamcode.Lib.STZLite.Drive.Chassis;
import org.firstinspires.ftc.teamcode.Lib.STZLite.Geometry.Pose;
import org.firstinspires.ftc.teamcode.Lib.STZLite.Geometry.Rotation;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;
import static dev.nextftc.extensions.pedro.PedroComponent.gyro;

import androidx.annotation.NonNull;

import java.util.ArrayList;
import java.util.List;


public class SuperChassis implements Subsystem {

    public static final SuperChassis INSTANCE = new SuperChassis();


    private Limelight3A limelight;

    private  double tx;
    private  double ty;
    private  double ta;

    private int lastDetectedId = -1;

    private List<LLResultTypes.FiducialResult> currentFiducials = new ArrayList<>();

    private final Pose robotPose = new Pose();
    private Command defaultCommand = new NullCommand();

    @Override
    public void initialize() {
        HardwareMap map = ActiveOpMode.hardwareMap();
        try {
            limelight = map.get(Limelight3A.class, VisionConstants.limelightName);
            limelight.setPollRateHz(100);
            limelight.start();
        } catch (Exception e) {
            ActiveOpMode.telemetry().addData("Limelight Error", e.getMessage());
        }
        try {
            Follower f = follower();
            if (f != null) {
                f.setStartingPose(Pose.kZero.toPedroPose());
            }
        } catch (Exception e) {
            ActiveOpMode.telemetry().addData("Follower Error", "Not initialized yet");
        }
    }

    @NonNull
    @Override
    public Command getDefaultCommand() {
        return defaultCommand;
    }
    public double getDistanceToTag() {
        if (!isLLConnected()) return 0.0;

        // 1. Get vertical offset (ty)
        double ty = getLLTy();
        double h_target = VisionConstants.TARGET_HEIGHT;
        double h_cam = VisionConstants.CAMERA_HEIGHT;
        double angle_mount = VisionConstants.CAMERA_ANGLE;
        // 2. Calculate angle sum (Camera Mount Angle + Target Offset)


        double angle_total_rad = Math.toRadians(angle_mount + ty);

        // Prevent division by zero if angle is perfectly 0
        if (Math.abs(Math.tan(angle_total_rad)) < 0.001) return 0.0;

        double distance = (h_target - h_cam) / Math.tan(angle_total_rad);

        return distance;

    }
    public void setDefaultCommand(Command command){
        this.defaultCommand = command;
    }
    @Override
    public void periodic() {
        if (limelight == null || !limelight.isConnected()) {
            clearFiducials();
            return;
        }

        try {
            limelight.updateRobotOrientation(getAngle().inRad);
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

                Pose3D botpose_mt2 = result.getBotpose_MT2();

                if (botpose_mt2 != null) {
                    double x = botpose_mt2.getPosition().x;
                    double y = botpose_mt2.getPosition().y;

                    robotPose.setX(x);
                    robotPose.setY(y);
                    robotPose.setRotation(getAngle().inRad);

                    try {
                        Follower f = follower();
                        if (f != null) {
                            f.setPose(robotPose.toPedroPose());
                        }
                    } catch (Exception e) {
                        // Follower not ready yet
                    }
                }

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

    private void clearFiducials() {
        if (this.currentFiducials != null) {
            this.currentFiducials.clear();
        }
        this.lastDetectedId = -1;
    }

    public SubsystemComponent asCOMPONENT() {
        return new SubsystemComponent(INSTANCE);
    }

    public Limelight3A getLIMELIGHT() {
        return limelight;
    }

    public Follower getFOLLOWER() {
        try {
            return follower();
        } catch (Exception e) {
            return null;
        }
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

    public boolean isLLConnected() {
        return limelight != null && limelight.isConnected();
    }

    private boolean validLLResult(LLResult result) {
        return result != null && result.isValid();
    }

    public Angle getAngle() {
        try {
            return gyro().get();
        } catch (Exception e) {
            return Angle.fromRad(0);
        }
    }

    public double getLLTx() {
        return tx;
    }

    public double getLLTy() {
        return ty;
    }

    public double getLLTa() {
        return ta;
    }

    public void switchLLPipeline(int pipe) {
        if (limelight != null) {
            limelight.pipelineSwitch(pipe);
        }
    }

    public void resetHeading() {
        try {
            Follower f = follower();
            if (f != null) {
                com.pedropathing.geometry.Pose pedroPose = f.poseTracker.getPose();
                Pose reset = new Pose(pedroPose.getX(), pedroPose.getY(), Rotation.kZero);
                f.poseTracker.setPose(reset.toPedroPose());
            }
        } catch (Exception e) {
            // Follower not ready
        }
    }

    public void stop() {

        try {
            if (follower() != null) {
                follower().setTeleOpDrive(0, 0, 0, false);
            }
        } catch (Exception e) {
            dev.nextftc.ftc.ActiveOpMode.telemetry().addData("SUPERCHASSIS ERROR", e.getMessage());
        }
    }
}

