package org.firstinspires.ftc.teamcode.Robot.Subsystems.LED;

/**
 * Constants for the REV Digital Indicator (REV-31-2010)
 *
 * This is a simple 2-color LED with green and red LEDs.
 * Colors available:
 * - Green only = Green
 * - Red only = Red
 * - Both on = Amber/Yellow
 * - Both off = Off
 */
public class LEDConstants {

    // ===== HARDWARE NAMES =====
    // The green LED should be on the lower digital channel
    // The red LED should be on the higher digital channel
    public static final String LED_GREEN_NAME = "led_green";
    public static final String LED_RED_NAME = "led_red";

    // ===== LED COLOR STATES =====
    public enum LEDColor {
        OFF,        // Both LEDs off
        GREEN,      // Green LED only
        RED,        // Red LED only
        AMBER       // Both LEDs on (green + red = amber/yellow)
    }

    // ===== TIMING =====
    public static final long SHOT_DISPLAY_TIME_MS = 300;   // How long to show shot color
    public static final long INTAKE_SUCCESS_TIME_MS = 200; // How long to show intake success
    public static final long STROBE_INTERVAL_MS = 100;     // Flash interval for strobe effect
}
