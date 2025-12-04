package org.firstinspires.ftc.teamcode.Robot.Subsystems.LED;

import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.hardware.HardwareManager;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.LED.LEDConstants.LEDColor;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.VisionConstants.BallColor;

/**
 * LED Subsystem for REV Digital Indicator (REV-31-2010)
 *
 * Controls a simple 2-color LED (green and red).
 * Supports solid colors and strobe/flashing effects.
 *
 * Color mapping for robot states:
 * - Intaking: Amber (both LEDs on)
 * - Spinning up green ball: Green flashing
 * - Spinning up purple ball: Red flashing
 * - Shot green ball: Green static
 * - Shot purple ball: Red static
 * - Ready: Green static
 * - Idle: Off
 */
public class LED implements Subsystem {

    public static final LED INSTANCE = new LED();

    // Hardware
    private com.qualcomm.robotcore.hardware.LED ledGreen;
    private com.qualcomm.robotcore.hardware.LED ledRed;
    private boolean hardwareAvailable = false;

    // Current state
    private LEDColor currentColor = LEDColor.OFF;
    private boolean isStrobing = false;
    private LEDColor strobeColor = LEDColor.OFF;

    // Timed pattern state
    private LEDColor timedColor = null;
    private LEDColor returnColor = LEDColor.OFF;
    private ElapsedTime timedColorTimer = new ElapsedTime();
    private long timedColorDuration = 0;

    // Strobe timing
    private ElapsedTime strobeTimer = new ElapsedTime();
    private boolean strobeState = false;  // true = on, false = off

    @Override
    public void initialize() {
        try {
            ledGreen = HardwareManager.getHardwareMap()
                    .get(com.qualcomm.robotcore.hardware.LED.class, LEDConstants.LED_GREEN_NAME);
            ledRed = HardwareManager.getHardwareMap()
                    .get(com.qualcomm.robotcore.hardware.LED.class, LEDConstants.LED_RED_NAME);
            hardwareAvailable = true;
            setColor(LEDColor.OFF);
        } catch (Exception e) {
            hardwareAvailable = false;
            ledGreen = null;
            ledRed = null;
        }
    }

    @Override
    public void periodic() {
        // Handle strobe effect
        if (isStrobing) {
            if (strobeTimer.milliseconds() >= LEDConstants.STROBE_INTERVAL_MS) {
                strobeTimer.reset();
                strobeState = !strobeState;
                if (strobeState) {
                    applyColor(strobeColor);
                } else {
                    applyColor(LEDColor.OFF);
                }
            }
        }

        // Handle timed colors
        if (timedColor != null) {
            if (timedColorTimer.milliseconds() >= timedColorDuration) {
                timedColor = null;
                isStrobing = false;
                setColor(returnColor);
            }
        }
    }

    // ===== COLOR CONTROL =====

    /**
     * Set a solid LED color
     */
    public void setColor(LEDColor color) {
        currentColor = color;
        isStrobing = false;
        timedColor = null;
        applyColor(color);
    }

    /**
     * Set a strobing/flashing LED color
     */
    public void setStrobe(LEDColor color) {
        currentColor = color;
        strobeColor = color;
        isStrobing = true;
        timedColor = null;
        strobeTimer.reset();
        strobeState = true;
        applyColor(color);
    }

    /**
     * Set a color for a limited time, then return to previous color
     */
    public void setColorTimed(LEDColor color, long durationMs) {
        returnColor = currentColor;
        timedColor = color;
        timedColorDuration = durationMs;
        timedColorTimer.reset();
        isStrobing = false;
        applyColor(color);
    }

    /**
     * Set a strobing color for a limited time
     */
    public void setStrobeTimed(LEDColor color, long durationMs) {
        returnColor = currentColor;
        timedColor = color;
        timedColorDuration = durationMs;
        timedColorTimer.reset();
        strobeColor = color;
        isStrobing = true;
        strobeTimer.reset();
        strobeState = true;
        applyColor(color);
    }

    /**
     * Apply color to the physical LEDs
     */
    private void applyColor(LEDColor color) {
        if (!hardwareAvailable) return;

        switch (color) {
            case GREEN:
                if (ledGreen != null) ledGreen.on();
                if (ledRed != null) ledRed.off();
                break;
            case RED:
                if (ledGreen != null) ledGreen.off();
                if (ledRed != null) ledRed.on();
                break;
            case AMBER:
                if (ledGreen != null) ledGreen.on();
                if (ledRed != null) ledRed.on();
                break;
            case OFF:
            default:
                if (ledGreen != null) ledGreen.off();
                if (ledRed != null) ledRed.off();
                break;
        }
    }

    // ===== CONVENIENCE METHODS FOR ROBOT STATES =====

    /**
     * Set LED to intaking state (amber)
     */
    public void setIntaking() {
        setColor(LEDColor.AMBER);
    }

    /**
     * Flash intake success briefly (green flash)
     */
    public void flashIntakeSuccess() {
        setColorTimed(LEDColor.GREEN, LEDConstants.INTAKE_SUCCESS_TIME_MS);
    }

    /**
     * Set LED to spinning up state (flashing ball color)
     * Green ball = green strobe, Purple ball = red strobe
     */
    public void setSpinningUp(BallColor ballColor) {
        switch (ballColor) {
            case GREEN:
                setStrobe(LEDColor.GREEN);
                break;
            case PURPLE:
                setStrobe(LEDColor.RED);
                break;
            default:
                setStrobe(LEDColor.AMBER);  // Unknown = amber strobe
                break;
        }
    }

    /**
     * Set LED to shot completed state (static ball color)
     * Green ball = green, Purple ball = red
     */
    public void setShotComplete(BallColor ballColor) {
        LEDColor color;
        switch (ballColor) {
            case GREEN:
                color = LEDColor.GREEN;
                break;
            case PURPLE:
                color = LEDColor.RED;
                break;
            default:
                color = LEDColor.AMBER;
                break;
        }
        setColorTimed(color, LEDConstants.SHOT_DISPLAY_TIME_MS);
    }

    /**
     * Set LED to idle state (off)
     */
    public void setIdle() {
        setColor(LEDColor.OFF);
    }

    /**
     * Set LED to ready state (dim green)
     */
    public void setReady() {
        setColor(LEDColor.GREEN);
    }

    /**
     * Set LED to spindexer full state (amber)
     */
    public void setFull() {
        setColor(LEDColor.AMBER);
    }

    /**
     * Set LED to error state (red)
     */
    public void setError() {
        setColor(LEDColor.RED);
    }

    // ===== STATE QUERIES =====

    public LEDColor getCurrentColor() {
        return currentColor;
    }

    public boolean isHardwareAvailable() {
        return hardwareAvailable;
    }

    public boolean isStrobing() {
        return isStrobing;
    }

    // ===== COMPONENT REGISTRATION =====

    public SubsystemComponent asCOMPONENT() {
        return new SubsystemComponent(INSTANCE);
    }
}
