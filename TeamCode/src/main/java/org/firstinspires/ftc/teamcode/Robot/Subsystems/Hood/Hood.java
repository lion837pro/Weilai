package org.firstinspires.ftc.teamcode.Robot.Subsystems.Hood;

/*
 * HOOD SUBSYSTEM - COMMENTED OUT
 * Using RPM-based distance shooting instead of hood angle adjustment.
 * To re-enable, uncomment this entire file.
 */

/*
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class Hood implements Subsystem {

    public static final Hood INSTANCE = new Hood();

    private Servo hoodServo;
    private double currentAngleDegrees = HoodConstants.DEFAULT_ANGLE_DEG;
    private double targetAngleDegrees = HoodConstants.DEFAULT_ANGLE_DEG;
    private Command defaultCommand = new NullCommand();

    @Override
    public void initialize() {
        try {
            hoodServo = ActiveOpMode.hardwareMap().get(Servo.class, HoodConstants.HOOD_SERVO_NAME);
            if (HoodConstants.SERVO_REVERSED) {
                hoodServo.setDirection(Servo.Direction.REVERSE);
            }
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

    public void setAngleDegrees(double degrees) {
        if (hoodServo == null) return;
        degrees = HoodConstants.clampAngle(degrees);
        targetAngleDegrees = degrees;
        currentAngleDegrees = degrees;
        double servoPosition = HoodConstants.degreesToServoPosition(degrees);
        hoodServo.setPosition(servoPosition);
    }

    public void setHoodForDistance(double distanceInches) {
        double angle = HoodConstants.calculateAngleForDistance(distanceInches);
        setAngleDegrees(angle);
    }

    public void adjustAngle(double deltaDegrees) {
        setAngleDegrees(currentAngleDegrees + deltaDegrees);
    }

    public void setClose() { setAngleDegrees(HoodConstants.ANGLE_PRESET_CLOSE); }
    public void setMid() { setAngleDegrees(HoodConstants.ANGLE_PRESET_MID); }
    public void setFar() { setAngleDegrees(HoodConstants.ANGLE_PRESET_FAR); }
    public void setDefault() { setAngleDegrees(HoodConstants.DEFAULT_ANGLE_DEG); }

    public double getAngleDegrees() { return currentAngleDegrees; }
    public double getTargetAngleDegrees() { return targetAngleDegrees; }
    public double getServoPosition() { return HoodConstants.degreesToServoPosition(currentAngleDegrees); }
    public boolean isInitialized() { return hoodServo != null; }

    private void updateTelemetry() {
        try {
            ActiveOpMode.telemetry().addData("--- HOOD ---", "");
            ActiveOpMode.telemetry().addData("Angle", "%.1f deg", currentAngleDegrees);
            ActiveOpMode.telemetry().addData("Servo Pos", "%.3f", getServoPosition());
            ActiveOpMode.telemetry().addData("Status", hoodServo != null ? "OK" : "NOT FOUND");
        } catch (Exception e) {}
    }

    public SubsystemComponent asCOMPONENT() {
        return new SubsystemComponent(INSTANCE);
    }
}
*/
