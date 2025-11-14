package org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Lib.STZLite.Drive.Chassis;
import org.firstinspires.ftc.teamcode.Lib.STZLite.Geometry.Pose;
import org.firstinspires.ftc.teamcode.Lib.STZLite.Geometry.Rotation;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;
import static dev.nextftc.extensions.pedro.PedroComponent.gyro;

import java.util.ArrayList;
import java.util.List;


public class SuperChassis implements Subsystem {

    public static final SuperChassis INSTANCE = new SuperChassis();

    private Chassis mecanum;
    private Limelight3A limelight;

    private  double tx;
    private  double ty;
    private  double ta;

    private int lastDetectedId = -1;

    private List<LLResultTypes.FiducialResult> currentFiducials = new ArrayList<>();

    private final Pose robotPose = new Pose();

    @Override
    public void initialize() {
        HardwareMap map = ActiveOpMode.hardwareMap();
        mecanum = new Chassis();
        mecanum.initialize();
        new PedroComponent(ChassisConstants::buildPedroPathing);
        limelight = map.get(Limelight3A.class, VisionConstants.limelightName);
        limelight.setPollRateHz(100);
        limelight.start();
        follower().setStartingPose(Pose.kZero.toPedroPose());
    }

    @Override
    public void periodic(){

        if(limelight.isConnected()){

            limelight.updateRobotOrientation(getAngle().inRad);
            LLResult result = limelight.getLatestResult();

            if(validLLResult(result)){

                long staleness = result.getStaleness();

                if(staleness < VisionConstants.STALENESS_THRESHOLD_MS){
                    telemetry.addData("Limelight Data", "Fresh (" + staleness + " ms)");
                    this.tx = result.getTx(); // How far left or right the target is (degrees)
                    this.ty = result.getTy(); // How far up or down the target is (degrees)
                    this.ta = result.getTa(); // How big the target looks (0%-100% of the image)

                    Pose3D botpose_mt2 = result.getBotpose_MT2();

                    if (botpose_mt2 != null) {
                        double x = botpose_mt2.getPosition().x;
                        double y = botpose_mt2.getPosition().y;

                        robotPose.setX(x);
                        robotPose.setY(y);
                        robotPose.setRotation(getAngle().inRad);

                        telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
                        follower().setPose(robotPose.toPedroPose());
                    }
                    this.currentFiducials = result.getFiducialResults();

                    if (this.currentFiducials != null && !this.currentFiducials.isEmpty()) {
                        this.lastDetectedId = this.currentFiducials.get(0).getFiducialId();
                        telemetry.addData("Viendo Tag (primero)", this.lastDetectedId);
                    } else {
                        this.lastDetectedId = -1;
                        telemetry.addData("Viendo Tag", "Ninguno");
                    }

                }
                //DATOS VIEJOS
                else{
                    telemetry.addData("Limelight Data", "Stale (" + staleness + " ms)");
                    if (this.currentFiducials != null) this.currentFiducials.clear();
                    this.lastDetectedId = -1;
                }

            } else {
                //RESULT NO VALIDO
                if (this.currentFiducials != null) this.currentFiducials.clear();
                this.lastDetectedId = -1;
            }
        } else {
            //LIMELIGHT DESCONECTADA
            if (this.currentFiducials != null) this.currentFiducials.clear();
            this.lastDetectedId = -1;
        }

    }

    public SubsystemComponent asCOMPONENT(){
        return new SubsystemComponent(INSTANCE);
    }

    public Limelight3A getLIMELIGHT(){
        return limelight;
    }

    public Follower getFOLLOWER(){
        return follower();
    }

    public boolean hasID(int id){
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

    public boolean isLLConnected(){
        return limelight.isConnected();
    }

    private boolean validLLResult(LLResult result){
        return result != null && result.isValid();
    }

    public Angle getAngle(){
        return gyro().get();
    }

    public double getLLTx(){
        return tx;
    }

    public double getLLTy(){
        return ty;
    }

    public double getLLTa(){
        return ta;
    }

    public void switchLLPipeline(int pipe){
        limelight.pipelineSwitch(pipe);
    }

    public void resetHeading(){
        com.pedropathing.geometry.Pose pedroPose = follower().poseTracker.getPose();
        Pose reset = new Pose(pedroPose.getX(), pedroPose.getY(), Rotation.kZero);
        follower().poseTracker.setPose(reset.toPedroPose());
    }

    public void stop(){
        mecanum.stop();
    }


}