package org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.Intake;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Shooter implements Subsystem {

    private VelocityProfileController controller;
    private MotorEx shooter1;
    private MotorEx shooter2;

    private boolean hasTarget = false;
    private boolean open = false;

    @Override
    public void initialize() {
        this.controller = new VelocityProfileController(
                ShooterConstants.kP,
                ShooterConstants.kS,
                ShooterConstants.kV);

        this.shooter1 = new MotorEx(ShooterConstants.motorName1);
        this.shooter2 = new MotorEx(ShooterConstants.motorName2);

        shooter1.floatMode();
        shooter2.floatMode();
    }

    @Override
    public void periodic() {

        if(hasTarget && !open){
            setPower(controller.calculate(getVelocity()));
        }

    }

    public double getVelocity(){
        return (shooter1.getVelocity() + shooter2.getVelocity()) / 2.0;
    }

    public void toVelocity(double velocity){
        this.hasTarget = true;
        this.open = false;
        controller.setTarget(velocity);
    }

    private void setPower(double power) {
        shooter1.setPower(power);
        shooter2.setPower(power);
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

}