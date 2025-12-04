package org.firstinspires.ftc.teamcode.Robot.Subsystems.LED;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;

/**
 * Constants for the REV Blinkin LED Driver (REV-31-2010)
 */
public class LEDConstants {

    // ===== HARDWARE NAME =====
    public static final String BLINKIN_NAME = "blinkin";

    // ===== LED PATTERNS FOR ROBOT STATES =====

    // Intake states
    public static final BlinkinPattern PATTERN_INTAKING = BlinkinPattern.BLUE;
    public static final BlinkinPattern PATTERN_INTAKE_SUCCESS = BlinkinPattern.AQUA;

    // Shooter spin-up states (flashing patterns for ball colors)
    public static final BlinkinPattern PATTERN_SPINUP_GREEN = BlinkinPattern.STROBE_GOLD;    // Flashing for green ball
    public static final BlinkinPattern PATTERN_SPINUP_PURPLE = BlinkinPattern.STROBE_BLUE;   // Flashing for purple ball
    public static final BlinkinPattern PATTERN_SPINUP_UNKNOWN = BlinkinPattern.STROBE_WHITE; // Flashing for unknown color

    // Shot completed states (static colors)
    public static final BlinkinPattern PATTERN_SHOT_GREEN = BlinkinPattern.GREEN;
    public static final BlinkinPattern PATTERN_SHOT_PURPLE = BlinkinPattern.VIOLET;
    public static final BlinkinPattern PATTERN_SHOT_UNKNOWN = BlinkinPattern.WHITE;

    // Idle/ready states
    public static final BlinkinPattern PATTERN_IDLE = BlinkinPattern.BLACK;      // LEDs off
    public static final BlinkinPattern PATTERN_READY = BlinkinPattern.DARK_GREEN; // Ready for action
    public static final BlinkinPattern PATTERN_FULL = BlinkinPattern.GOLD;        // Spindexer full (3 balls)

    // Error/warning states
    public static final BlinkinPattern PATTERN_ERROR = BlinkinPattern.RED;
    public static final BlinkinPattern PATTERN_WARNING = BlinkinPattern.ORANGE;

    // ===== TIMING =====
    public static final long SHOT_DISPLAY_TIME_MS = 300;  // How long to show shot color
    public static final long INTAKE_SUCCESS_TIME_MS = 200; // How long to show intake success
}
