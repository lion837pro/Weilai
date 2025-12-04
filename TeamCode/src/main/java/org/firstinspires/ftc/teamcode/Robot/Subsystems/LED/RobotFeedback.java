package org.firstinspires.ftc.teamcode.Robot.Subsystems.LED;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.VisionConstants.BallColor;

/**
 * Robot Feedback utility class
 *
 * Provides combined LED and gamepad rumble feedback for robot events.
 * Call these methods when events occur to provide driver feedback.
 */
public class RobotFeedback {

    // Gamepad rumble durations (milliseconds)
    public static final int RUMBLE_INTAKE_SUCCESS = 150;
    public static final int RUMBLE_SHOT_FIRED = 200;
    public static final int RUMBLE_SPINDEXER_FULL = 400;

    // Rumble intensities (0.0 to 1.0)
    public static final double RUMBLE_LIGHT = 0.3;
    public static final double RUMBLE_MEDIUM = 0.6;
    public static final double RUMBLE_STRONG = 1.0;

    private final LED led;
    private Gamepad gamepad1;
    private Gamepad gamepad2;

    public RobotFeedback(LED led) {
        this.led = led;
    }

    /**
     * Set gamepads for rumble feedback
     */
    public void setGamepads(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    /**
     * Call when intake mode is started
     */
    public void onIntakeStart() {
        if (led != null) {
            led.setIntaking();
        }
    }

    /**
     * Call when a ball is successfully intaked
     */
    public void onIntakeSuccess() {
        // LED feedback
        if (led != null) {
            led.flashIntakeSuccess();
        }

        // Gamepad rumble
        rumbleBoth(RUMBLE_MEDIUM, RUMBLE_INTAKE_SUCCESS);
    }

    /**
     * Call when intake stops
     */
    public void onIntakeStop() {
        if (led != null) {
            led.setIdle();
        }
    }

    /**
     * Call when shooter is spinning up
     */
    public void onShooterSpinUp(BallColor nextBallColor) {
        if (led != null) {
            led.setSpinningUp(nextBallColor);
        }
    }

    /**
     * Call when a ball is shot
     */
    public void onBallShot(BallColor ballColor) {
        // LED feedback
        if (led != null) {
            led.setShotComplete(ballColor);
        }

        // Gamepad rumble
        rumbleBoth(RUMBLE_STRONG, RUMBLE_SHOT_FIRED);
    }

    /**
     * Call when shooter sequence stops
     */
    public void onShooterStop() {
        if (led != null) {
            led.setIdle();
        }
    }

    /**
     * Call when spindexer becomes full
     */
    public void onSpindexerFull() {
        // LED feedback
        if (led != null) {
            led.setFull();
        }

        // Long rumble to alert driver
        rumbleBoth(RUMBLE_STRONG, RUMBLE_SPINDEXER_FULL);
    }

    /**
     * Call when spindexer becomes empty
     */
    public void onSpindexerEmpty() {
        if (led != null) {
            led.setReady();
        }
    }

    /**
     * Set LED to ready state
     */
    public void setReady() {
        if (led != null) {
            led.setReady();
        }
    }

    /**
     * Set LED to idle state
     */
    public void setIdle() {
        if (led != null) {
            led.setIdle();
        }
    }

    // ===== RUMBLE HELPERS =====

    /**
     * Rumble both gamepads
     */
    public void rumbleBoth(double intensity, int durationMs) {
        rumbleGamepad(gamepad1, intensity, durationMs);
        rumbleGamepad(gamepad2, intensity, durationMs);
    }

    /**
     * Rumble gamepad 1 only
     */
    public void rumbleGamepad1(double intensity, int durationMs) {
        rumbleGamepad(gamepad1, intensity, durationMs);
    }

    /**
     * Rumble gamepad 2 only
     */
    public void rumbleGamepad2(double intensity, int durationMs) {
        rumbleGamepad(gamepad2, intensity, durationMs);
    }

    private void rumbleGamepad(Gamepad gamepad, double intensity, int durationMs) {
        if (gamepad != null) {
            gamepad.rumble(intensity, intensity, durationMs);
        }
    }

    /**
     * Asymmetric rumble (different left/right)
     */
    public void rumbleBothAsymmetric(double leftIntensity, double rightIntensity, int durationMs) {
        if (gamepad1 != null) {
            gamepad1.rumble(leftIntensity, rightIntensity, durationMs);
        }
        if (gamepad2 != null) {
            gamepad2.rumble(leftIntensity, rightIntensity, durationMs);
        }
    }
}
