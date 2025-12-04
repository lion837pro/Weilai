package org.firstinspires.ftc.teamcode.Robot.Subsystems.LED;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.hardware.HardwareManager;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive.VisionConstants.BallColor;

/**
 * LED Subsystem for REV Blinkin LED Driver (REV-31-2010)
 *
 * Controls LEDs based on robot state:
 * - Intaking: Blue
 * - Spinning up shooter: Flashing ball color (green or purple)
 * - Ball shot: Static ball color
 */
public class LED implements Subsystem {

    public static final LED INSTANCE = new LED();

    // Hardware
    private RevBlinkinLedDriver blinkin;
    private boolean hardwareAvailable = false;

    // State tracking
    private BlinkinPattern currentPattern = LEDConstants.PATTERN_IDLE;
    private BlinkinPattern requestedPattern = LEDConstants.PATTERN_IDLE;

    // Timed pattern state (for temporary displays like shot confirmation)
    private BlinkinPattern timedPattern = null;
    private BlinkinPattern returnPattern = LEDConstants.PATTERN_IDLE;
    private ElapsedTime timedPatternTimer = new ElapsedTime();
    private long timedPatternDuration = 0;

    @Override
    public void initialize() {
        try {
            blinkin = HardwareManager.getHardwareMap()
                    .get(RevBlinkinLedDriver.class, LEDConstants.BLINKIN_NAME);
            hardwareAvailable = true;
            setPattern(LEDConstants.PATTERN_IDLE);
        } catch (Exception e) {
            hardwareAvailable = false;
            blinkin = null;
        }
    }

    @Override
    public void periodic() {
        // Handle timed patterns
        if (timedPattern != null) {
            if (timedPatternTimer.milliseconds() >= timedPatternDuration) {
                // Timed pattern expired, return to previous pattern
                timedPattern = null;
                applyPattern(returnPattern);
            }
        }
    }

    // ===== PATTERN CONTROL =====

    /**
     * Set the LED pattern
     */
    public void setPattern(BlinkinPattern pattern) {
        requestedPattern = pattern;
        timedPattern = null;  // Cancel any timed pattern
        applyPattern(pattern);
    }

    /**
     * Set a pattern for a limited time, then return to previous pattern
     */
    public void setPatternTimed(BlinkinPattern pattern, long durationMs) {
        returnPattern = currentPattern;
        timedPattern = pattern;
        timedPatternDuration = durationMs;
        timedPatternTimer.reset();
        applyPattern(pattern);
    }

    private void applyPattern(BlinkinPattern pattern) {
        if (hardwareAvailable && blinkin != null && pattern != currentPattern) {
            currentPattern = pattern;
            blinkin.setPattern(pattern);
        }
    }

    // ===== CONVENIENCE METHODS FOR ROBOT STATES =====

    /**
     * Set LED to intaking state (blue)
     */
    public void setIntaking() {
        setPattern(LEDConstants.PATTERN_INTAKING);
    }

    /**
     * Flash intake success briefly
     */
    public void flashIntakeSuccess() {
        setPatternTimed(LEDConstants.PATTERN_INTAKE_SUCCESS, LEDConstants.INTAKE_SUCCESS_TIME_MS);
    }

    /**
     * Set LED to spinning up state (flashing ball color)
     */
    public void setSpinningUp(BallColor ballColor) {
        switch (ballColor) {
            case GREEN:
                setPattern(LEDConstants.PATTERN_SPINUP_GREEN);
                break;
            case PURPLE:
                setPattern(LEDConstants.PATTERN_SPINUP_PURPLE);
                break;
            default:
                setPattern(LEDConstants.PATTERN_SPINUP_UNKNOWN);
                break;
        }
    }

    /**
     * Set LED to shot completed state (static ball color)
     */
    public void setShotComplete(BallColor ballColor) {
        BlinkinPattern pattern;
        switch (ballColor) {
            case GREEN:
                pattern = LEDConstants.PATTERN_SHOT_GREEN;
                break;
            case PURPLE:
                pattern = LEDConstants.PATTERN_SHOT_PURPLE;
                break;
            default:
                pattern = LEDConstants.PATTERN_SHOT_UNKNOWN;
                break;
        }
        setPatternTimed(pattern, LEDConstants.SHOT_DISPLAY_TIME_MS);
    }

    /**
     * Set LED to idle state (off)
     */
    public void setIdle() {
        setPattern(LEDConstants.PATTERN_IDLE);
    }

    /**
     * Set LED to ready state
     */
    public void setReady() {
        setPattern(LEDConstants.PATTERN_READY);
    }

    /**
     * Set LED to spindexer full state
     */
    public void setFull() {
        setPattern(LEDConstants.PATTERN_FULL);
    }

    /**
     * Set LED to error state
     */
    public void setError() {
        setPattern(LEDConstants.PATTERN_ERROR);
    }

    // ===== STATE QUERIES =====

    public BlinkinPattern getCurrentPattern() {
        return currentPattern;
    }

    public boolean isHardwareAvailable() {
        return hardwareAvailable;
    }

    // ===== COMPONENT REGISTRATION =====

    public SubsystemComponent asCOMPONENT() {
        return new SubsystemComponent(INSTANCE);
    }
}
