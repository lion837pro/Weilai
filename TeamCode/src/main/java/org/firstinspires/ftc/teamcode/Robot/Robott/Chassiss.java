package org.firstinspires.ftc.teamcode.Robot.Robott;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Lib.STZLite.Geometry.Pose;
import org.firstinspires.ftc.teamcode.Lib.STZLite.Geometry.Rotation;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.SuperChassis;
import org.firstinspires.ftc.teamcode.Robot.Robott.Subsystems2.ChassisConstants2;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.units.Angle;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

public class Chassiss extends SuperChassis implements Subsystem {

    public static final Chassiss INSTANCE = new Chassiss();

    // Motors
    private MotorEx fl, fr, bl, br;

    // IMU
    private IMU imu;

    // State
    private Pose currentPose = new Pose();
    private Command defaultCommand = new NullCommand();

    // Drive mode
    private boolean fieldCentric = false;

    @Override
    public void initialize() {
        // Initialize motors
        fl = new MotorEx(ChassisConstants2.flName);
        fr = new MotorEx(ChassisConstants2.frName);
        bl = new MotorEx(ChassisConstants2.blName);
        br = new MotorEx(ChassisConstants2.brName);

        // Set directions
        fl.setDirection(ChassisConstants2.flDirection.ordinal());
        fr.setDirection(ChassisConstants2.frDirection.ordinal());
        bl.setDirection(ChassisConstants2.blDirection.ordinal());
        br.setDirection(ChassisConstants2.brDirection.ordinal());

        // Set zero power behavior
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize IMU
        imu = ActiveOpMode.hardwareMap().get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        ChassisConstants2.LOGO_FACING_DIR,
                        ChassisConstants2.USB_FACING_DIR
                )
        );
        imu.initialize(parameters);

        // Reset encoders
        resetEncoders();
    }

    @Override
    public void periodic() {
        // Update heading from IMU
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        currentPose.setRotation(heading);

        // Simple odometry using encoder counts (you can enhance this)
        // For now, just track heading
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

    // ========== DRIVE METHODS ==========

    /**
     * Main drive method for mecanum wheels
     * @param forward Forward/backward movement (-1 to 1)
     * @param strafe Left/right movement (-1 to 1)
     * @param turn Rotation (-1 to 1)
     * @param fieldCentric Whether to use field-centric control
     */
    public void drive(double forward, double strafe, double turn, boolean fieldCentric) {
        double x = strafe;
        double y = -forward; // Inverted because gamepad Y is negative up
        double rx = turn;

        // Field-centric transformation
        if (fieldCentric) {
            double heading = getHeading().inRad;
            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
            x = rotX;
            y = rotY;
        }

        // Calculate wheel powers
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        double flPower = (y + x + rx) / denominator;
        double blPower = (y - x + rx) / denominator;
        double frPower = (y - x - rx) / denominator;
        double brPower = (y + x - rx) / denominator;

        // Set motor powers
        fl.setPower(flPower);
        bl.setPower(blPower);
        fr.setPower(frPower);
        br.setPower(brPower);
    }

    /**
     * Stop all motors
     */
    public void stop() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    /**
     * Set power to individual wheels (for advanced control)
     */
    public void setWheelPowers(double flPower, double frPower, double blPower, double brPower) {
        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
    }

    // ========== IMU METHODS ==========

    /**
     * Get current heading as Angle
     */
    public Angle getHeading() {
        return Angle.fromRad(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }

    /**
     * Get current heading as Rotation
     */
    public Rotation getHeadingRotation() {
        return new Rotation(getHeading().inRad);
    }

    /**
     * Reset the heading to zero
     */
    public void resetHeading() {
        imu.resetYaw();
        currentPose.setRotation(0);
    }

    /**
     * Set heading to a specific angle
     */
    public void setHeading(double radians) {
        // This requires tracking an offset since IMU can't be set directly
        // For now, we'll just reset - you can enhance this
        resetHeading();
    }

    // ========== ODOMETRY METHODS ==========

    public Pose getPose() {
        return currentPose;
    }

    public void setPose(Pose pose) {
        this.currentPose = pose;
    }

    // ========== MOTOR CONFIGURATION ==========

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        fl.setZeroPowerBehavior(behavior);
        fr.setZeroPowerBehavior(behavior);
        bl.setZeroPowerBehavior(behavior);
        br.setZeroPowerBehavior(behavior);
    }

    public void setBrakeMode() {
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setCoastMode() {
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    private void resetEncoders() {
        fl.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // ========== GETTERS ==========

    public MotorEx getFrontLeft() { return fl; }
    public MotorEx getFrontRight() { return fr; }
    public MotorEx getBackLeft() { return bl; }
    public MotorEx getBackRight() { return br; }

    public boolean isFieldCentric() { return fieldCentric; }
    public void setFieldCentric(boolean fieldCentric) { this.fieldCentric = fieldCentric; }
}