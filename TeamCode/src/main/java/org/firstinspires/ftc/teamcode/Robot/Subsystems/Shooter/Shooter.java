package org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake.Intake;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Shooter implements Subsystem   {

    public static final Shooter INSTANCE = new Shooter();
    MotorEx Sh;

    @Override
    public void initialize() {
        Sh = new MotorEx("Sh");

    }
    @Override
    public void periodic() {
    }
    public void MoveSh(double speed) {
        Sh.setPower(speed);
    }
    public SubsystemComponent asCOMPONENT(){return new SubsystemComponent(INSTANCE);}
}
