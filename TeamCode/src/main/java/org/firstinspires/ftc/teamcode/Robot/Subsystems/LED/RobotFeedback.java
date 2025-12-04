package org.firstinspires.ftc.teamcode.Robot.Subsystems.LED;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Hardware.REV312010;
import org.firstinspires.ftc.teamcode.Robot.Hardware.REV312010.LEDState;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.VisionConstants.BallColor;

/**
 * Robot Feedback utility class
 *
 * Provides combined LED and gamepad rumble feedback for robot events.
 * Uses REV312010 (REV-31-2010) 2-color LED indicator.
 *
 * Color mapping:
 * - Intaking: Amber
 * - Spinning up green ball: Green (flashing)
 * - Spinning up purple ball: Red (flashing)
 * - Shot green ball: Green (static)
 * - Shot purple ball: Red (static)
 * - Ready: Green
 * - Idle
 * : Off
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

    // Strobe timing
    private static final long STROBE_INTERVAL_MS = 100;

    private final REV312010 led;
    private Gamepad gamepad1;
    private Gamepad gamepad2;

    // Strobe state
    private boolean isStrobing = false;
    private LEDState strobeColor = LEDState.kOFF;
    private boolean strobeOn = false;
    private ElapsedTime strobeTimer = new ElapsedTime();

    public RobotFeedback(REV312010 led) {
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
     * Call this in the periodic/update loop to handle strobe effect
     */
    public void update() {
        if (isStrobing && led != null) {
            if (strobeTimer.milliseconds() >= STROBE_INTERVAL_MS) {
                strobeTimer.reset();
                strobeOn = !strobeOn;
                led.set(strobeOn ? strobeColor : LEDState.kOFF);
            }
        }
    }

    /**
     * Call when intake mode is started
     */
    public void onIntakeStart() {
        isStrobing = false;
        if (led != null) {
            led.set(LEDState.kAMBER);
        }
    }

    /**
     * Call when a ball is successfully intaked
     */
    public void onIntakeSuccess() {
        // LED feedback - brief green flash
        isStrobing = false;
        if (led != null) {
            led.set(LEDState.kGREEN);
        }

        // Gamepad rumble
        rumbleBoth(RUMBLE_MEDIUM, RUMBLE_INTAKE_SUCCESS);
    }

    /**
     * Call when intake stops
     */
    public void onIntakeStop() {
        isStrobing = false;
        if (led != null) {
            led.set(LEDState.kOFF);
        }
    }

    /**
     * Call when shooter is spinning up
     */
    public void onShooterSpinUp(BallColor nextBallColor) {
        if (led != null) {
            // Start strobing in ball color
            isStrobing = true;
            strobeTimer.reset();
            strobeOn = true;

            switch (nextBallColor) {
                case GREEN:
                    strobeColor = LEDState.kGREEN;
                    break;
                case PURPLE:
                    strobeColor = LEDState.kRED;
                    break;
                default:
                    strobeColor = LEDState.kAMBER;
                    break;
            }
            led.set(strobeColor);
        }
    }

    /**
     * Call when a ball is shot
     */
    public void onBallShot(BallColor ballColor) {
        // LED feedback - static ball color
        isStrobing = false;
        if (led != null) {
            switch (ballColor) {
                case GREEN:
                    led.set(LEDState.kGREEN);
                    break;
                case PURPLE:
                    led.set(LEDState.kRED);
                    break;
                default:
                    led.set(LEDState.kAMBER);
                    break;
            }
        }

        // Gamepad rumble
        rumbleBoth(RUMBLE_STRONG, RUMBLE_SHOT_FIRED);
    }

    /**
     * Call when shooter sequence stops
     */
    public void onShooterStop() {
        isStrobing = false;
        if (led != null) {
            led.set(LEDState.kOFF);
        }
    }

    /**
     * Call when spindexer becomes full
     */
    public void onSpindexerFull() {
        // LED feedback
        isStrobing = false;
        if (led != null) {
            led.set(LEDState.kAMBER);
        }

        // Long rumble to alert driver
        rumbleBoth(RUMBLE_STRONG, RUMBLE_SPINDEXER_FULL);
    }

    /**
     * Call when spindexer becomes empty
     */
    public void onSpindexerEmpty() {
        isStrobing = false;
        if (led != null) {
            led.set(LEDState.kGREEN);
        }
    }

    /**
     * Set LED to ready state
     */
    public void setReady() {
        isStrobing = false;
        if (led != null) {
            led.set(LEDState.kGREEN);
        }
    }

    /**
     * Set LED to idle state
     */
    public void setIdle() {
        isStrobing = false;
        if (led != null) {
            led.set(LEDState.kOFF);
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
