package org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

import org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Controller.VelocityProfileController;
import org.firstinspires.ftc.teamcode.Lib.STZLite.Math.Intervals.Interval;

public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();

    private VelocityProfileController controller;
     MotorEx Sh1;
     MotorEx Sh2    ;




    private boolean hasTarget = false;
    private boolean open = false;
    private double currentPower = 0;


    private Command defaultCommand = new NullCommand();

    @Override
    public void initialize() {
        this.controller = new VelocityProfileController(
                ShooterConstants.kP,
                ShooterConstants.kS,
                ShooterConstants.kV);

        this.Sh1 = new MotorEx(ShooterConstants.shootername1);
        this.Sh2 = new MotorEx(ShooterConstants.shootername2).reversed();


        Sh1.floatMode();
        Sh2.floatMode();
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

        if(hasTarget && !open){
            setPower(controller.calculate(getVelocity()));
        }
        try {
            ActiveOpMode.telemetry().addData("--- SHOOTER DEBUG ---", "");

            // A. State
            ActiveOpMode.telemetry().addData("Mode", hasTarget ? "PID (Auto)" : "Manual (Power)");
            ActiveOpMode.telemetry().addData("Target Velocity", hasTarget ? controller.getTarget() : 0);
            ActiveOpMode.telemetry().addData("Current Velocity", getVelocity());

            // B. Output
            ActiveOpMode.telemetry().addData("Applied Power", currentPower);

            // C. The "Get with PID Values" (Verify constants are not 0!)
            ActiveOpMode.telemetry().addData("Constants", "kP=%.5f, kS=%.5f, kV=%.5f",
                    ShooterConstants.kP, ShooterConstants.kS, ShooterConstants.kV);

            ActiveOpMode.telemetry().update(); // Force update to see it live
        } catch (Exception e) {
            // Failsafe if telemetry isn't ready
        }
    }


    public double getVelocity(){
        return (Sh1.getVelocity() + Sh2.getVelocity()) / 2.0;
    }

    public void toVelocity(double velocity){
        this.hasTarget = true;
        this.open = false;
        controller.setTarget(velocity);
    }

    private void setPower(double power) {
        this.currentPower = power;
        Sh1.setPower(power);
        Sh2.setPower(power);
    }

    public void set(double power) {
        hasTarget = false;
        open = true;
        setPower(power);
    }

    public void stop() {
        hasTarget = false;
        open = true;
        setPower(0);
    }

    public boolean atSetpoint() {
        if (!hasTarget) {
            return false;
        }

        double target = controller.getTarget();

        double minLimit = target - ShooterConstants.velocityTolerance;
        double maxLimit = target + ShooterConstants.velocityTolerance;

        return Interval.isInRange(getVelocity(), minLimit, maxLimit);
    }
    public SubsystemComponent asCOMPONENT(){return new SubsystemComponent(INSTANCE);}

}