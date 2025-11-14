package org.firstinspires.ftc.teamcode.Intake;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Intake implements Subsystem {

    public static final Intake INSTANCE = new Intake();
    MotorEx In;

    @Override
    public void initialize() {
        In  = new MotorEx("In");


    }

    @Override
    public void periodic() {




    }

    public void MoveIn(double speed){
        In.setPower(speed);
    }

}
