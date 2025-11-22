package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Shooter.ShooterConstants;

@TeleOp(name = "Shooter Tuner", group = "Tuning")
public class ShooterTuner extends OpMode {

    private Shooter shooter;

    // Tunable constants (adjust with gamepad)
    private double kP = ShooterConstants.kP;
    private double kS = ShooterConstants.kS;
    private double kV = ShooterConstants.kV;

    private double targetRPM = 3000;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastY = false;
    private boolean lastA = false;

    private enum TuningMode {
        KP, KS, KV
    }

    private TuningMode currentMode = TuningMode.KV;

    @Override
    public void init() {
        shooter = Shooter.INSTANCE;
        shooter.initialize();

        telemetry.addLine("=== SHOOTER TUNER ===");
        telemetry.addLine("Controls:");
        telemetry.addLine("A/Y: Change RPM target");
        telemetry.addLine("X: Toggle tuning mode (kP/kS/kV)");
        telemetry.addLine("DPad Up/Down: Adjust selected constant");
        telemetry.addLine("DPad Left/Right: Fine adjust");
        telemetry.addLine("Right Bumper: Run shooter");
        telemetry.addLine("Left Bumper: Stop shooter");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Handle mode switching
        if (gamepad1.x && !lastDpadLeft) {
            currentMode = TuningMode.values()[(currentMode.ordinal() + 1) % 3];
        }

        // Handle constant adjustment
        double coarseStep = getCoarseStep();
        double fineStep = getFineStep();

        if (gamepad1.dpad_up && !lastDpadUp) {
            adjustConstant(coarseStep);
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            adjustConstant(-coarseStep);
        }
        if (gamepad1.dpad_right && !lastDpadRight) {
            adjustConstant(fineStep);
        }
        if (gamepad1.dpad_left && !lastDpadLeft) {
            adjustConstant(-fineStep);
        }

        // Handle RPM adjustment
        if (gamepad1.y && !lastY) {
            targetRPM += 500;
            if (targetRPM > 6000) targetRPM = 6000;
        }
        if (gamepad1.a && !lastA) {
            targetRPM -= 500;
            if (targetRPM < 0) targetRPM = 0;
        }

        // Apply constants to shooter (live update)
        updateShooterConstants();

        // Control shooter
        if (gamepad1.right_bumper) {
            double targetTPS = ShooterConstants.rpmToTicksPerSecond(targetRPM);
            shooter.toVelocity(targetTPS);
        } else if (gamepad1.left_bumper) {
            shooter.stop();
        }

        // Update button states
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;
        lastY = gamepad1.y;
        lastA = gamepad1.a;

        // Display telemetry
        displayTelemetry();
    }

    private void adjustConstant(double delta) {
        switch (currentMode) {
            case KP:
                kP += delta;
                if (kP < 0) kP = 0;
                break;
            case KS:
                kS += delta;
                if (kS < 0) kS = 0;
                if (kS > 1.0) kS = 1.0;
                break;
            case KV:
                kV += delta;
                if (kV < 0) kV = 0;
                break;
        }
    }

    private double getCoarseStep() {
        switch (currentMode) {
            case KP: return 0.001;
            case KS: return 0.01;
            case KV: return 0.0001;
            default: return 0.001;
        }
    }

    private double getFineStep() {
        switch (currentMode) {
            case KP: return 0.0001;
            case KS: return 0.001;
            case KV: return 0.00001;
            default: return 0.0001;
        }
    }

    private void updateShooterConstants() {
        // This requires modifying your VelocityProfileController to allow runtime updates
        // For now, we'll just display what the values should be
        // You'll need to add a setGains() method to your controller
    }

    private void displayTelemetry() {
        telemetry.addLine("=== SHOOTER TUNER ===");
        telemetry.addLine();

        // Current mode indicator
        telemetry.addLine("Tuning Mode (X to change):");
        telemetry.addData("  > kP", currentMode == TuningMode.KP ? "***" : "");
        telemetry.addData("  > kS", currentMode == TuningMode.KS ? "***" : "");
        telemetry.addData("  > kV", currentMode == TuningMode.KV ? "***" : "");
        telemetry.addLine();

        // Constants
        telemetry.addData("kP", "%.6f", kP);
        telemetry.addData("kS", "%.6f", kS);
        telemetry.addData("kV", "%.6f", kV);
        telemetry.addLine();

        // Target vs Actual
        double targetTPS = ShooterConstants.rpmToTicksPerSecond(targetRPM);
        double actualTPS = shooter.getVelocity();
        double actualRPM = ShooterConstants.ticksPerSecondToRPM(actualTPS);

        telemetry.addData("Target RPM (A/Y)", "%.0f", targetRPM);
        telemetry.addData("Target TPS", "%.0f", targetTPS);
        telemetry.addData("Actual TPS", "%.0f", actualTPS);
        telemetry.addData("Actual RPM", "%.0f", actualRPM);
        telemetry.addData("Error", "%.0f tps", targetTPS - actualTPS);
        telemetry.addLine();

        // Status
        telemetry.addData("At Setpoint?", shooter.atSetpoint() ? "YES" : "NO");
        telemetry.addLine();

        // Copy-paste ready constants
        telemetry.addLine("=== COPY THESE TO ShooterConstants.java ===");
        telemetry.addLine(String.format("public static final double kP = %.6f;", kP));
        telemetry.addLine(String.format("public static final double kS = %.6f;", kS));
        telemetry.addLine(String.format("public static final double kV = %.6f;", kV));

        telemetry.update();
    }
}