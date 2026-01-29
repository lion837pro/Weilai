package org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;

import androidx.annotation.NonNull;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

public class Intake implements Subsystem {

    public static final Intake INSTANCE = new Intake();

    private MotorEx motor;
    private double currentPower = 0;
    private Command defaultCommand = new NullCommand();

    @Override
    public void initialize() {
        try {
            motor = new MotorEx(IntakeConstants.intakename);
            if (IntakeConstants.intakeinverted) {
                motor.reversed();
            }
            // Power efficiency: use float mode - intake doesn't need holding torque
            motor.floatMode();
        } catch (Exception e) {
            motor = null;
            ActiveOpMode.telemetry().addData("Intake Error", "Motor not found");
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
        // Telemetry handled externally to reduce overhead
    }

    /**
     * Run intake at specified power (-1.0 to 1.0)
     * Positive = intake, Negative = outtake
     */
    public void run(double power) {
        if (motor == null) return;
        this.currentPower = power;
        motor.setPower(power);
    }

    /**
     * Stop the intake motor
     */
    public void stop() {
        run(0);
    }

    /**
     * Legacy method for compatibility
     */
    public void MoveIn(double speed) {
        run(speed);
    }

    /**
     * Get current motor power
     */
    public double getCurrentPower() {
        return currentPower;
    }

    /**
     * Check if intake is running
     */
    public boolean isRunning() {
        return Math.abs(currentPower) > 0.05;
    }

    public SubsystemComponent asCOMPONENT() {
        return new SubsystemComponent(INSTANCE);
    }
}
