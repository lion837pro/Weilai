package org.firstinspires.ftc.teamcode.Robot.Emergencia;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class SimpleMecanumChassis implements Subsystem {

    public static final SimpleMecanumChassis INSTANCE = new SimpleMecanumChassis();

    // Motor names
    private static final String FL_MOTOR = "fl";
    private static final String FR_MOTOR = "fr";
    private static final String BL_MOTOR = "bl";
    private static final String BR_MOTOR = "br";

    // Motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // IMU
    private IMU imu;

    // Drive settings
    private boolean fieldCentric = true;
    private double powerMultiplier = 1.0;
    private double headingOffset = 0.0;
    private Command defaultCommand = new NullCommand();

    @Override
    public void initialize() {
        // Initialize motors
        frontLeft = ActiveOpMode.hardwareMap().get(DcMotor.class, FL_MOTOR);
        frontRight = ActiveOpMode.hardwareMap().get(DcMotor.class, FR_MOTOR);
        backLeft = ActiveOpMode.hardwareMap().get(DcMotor.class, BL_MOTOR);
        backRight = ActiveOpMode.hardwareMap().get(DcMotor.class, BR_MOTOR);

        // Set motor directions (adjust these based on your robot)
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set zero power behavior
    setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set run modes
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize IMU
        imu = ActiveOpMode.hardwareMap().get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(parameters);

        // Reset heading
        resetHeading();
    }
    @NonNull
    @Override
    public Command getDefaultCommand() {
        return defaultCommand;
    }
    public void setDefaultCommand(Command command){
        this.defaultCommand = command;
    }
    @Override
    public void periodic() {
        // Update telemetry
        ActiveOpMode.telemetry().addData("Drive Mode", fieldCentric ? "Field-Centric" : "Robot-Centric");
        ActiveOpMode.telemetry().addData("Heading", "%.1f deg", getHeading());
        ActiveOpMode.telemetry().addData("Power Multiplier", "%.1f%%", powerMultiplier * 100);
    }

    /**
     * Main drive method for mecanum wheels
     * @param forward Forward/backward movement (-1 to 1)
     * @param strafe Left/right movement (-1 to 1)
     * @param turn Rotation (-1 to 1)
     */
    public void drive(double forward, double strafe, double turn) {
        double x = strafe;
        double y = forward;
        double rx = turn;

        // Field-centric transformation
        if (fieldCentric) {
            double heading = Math.toRadians(getHeading());
            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
            x = rotX;
            y = rotY;
        }

        // Calculate wheel powers (mecanum kinematics)
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        // Apply power multiplier
        frontLeftPower *= powerMultiplier;
        backLeftPower *= powerMultiplier;
        frontRightPower *= powerMultiplier;
        backRightPower *= powerMultiplier;

        // Set motor powers
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    /**
     * Stop all motors
     */
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    /**
     * Get current heading in degrees
     */
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - headingOffset;
    }

    /**
     * Get raw IMU heading without offset
     */
    public double getRawHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /**
     * Reset the heading to zero
     */
    public void resetHeading() {
        headingOffset = getRawHeading();
    }

    /**
     * Toggle field-centric mode
     */
    public void toggleFieldCentric() {
        fieldCentric = !fieldCentric;
    }

    /**
     * Set field-centric mode
     */
    public void setFieldCentric(boolean enabled) {
        fieldCentric = enabled;
    }

    /**
     * Check if field-centric mode is enabled
     */
    public boolean isFieldCentric() {
        return fieldCentric;
    }

    /**
     * Set power multiplier (for slow mode)
     */
    public void setPowerMultiplier(double multiplier) {
        powerMultiplier = Math.max(0.1, Math.min(1.0, multiplier));
    }

    /**
     * Get current power multiplier
     */
    public double getPowerMultiplier() {
        return powerMultiplier;
    }

    /**
     * Set zero power behavior for all motors
     */
    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        frontLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
        backRight.setZeroPowerBehavior(behavior);
    }

    /**
     * Set brake mode
     */
    public void setBrakeMode() {
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Set coast mode
     */
    public void setCoastMode() {
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Get motor powers for telemetry
     */
    public double[] getMotorPowers() {
        return new double[] {
                frontLeft.getPower(),
                frontRight.getPower(),
                backLeft.getPower(),
                backRight.getPower()
        };
    }

    /**
     * Get motor currents for telemetry
     */


    public SubsystemComponent asCOMPONENT() {
        return new SubsystemComponent(INSTANCE);
    }
}