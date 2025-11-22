package org.firstinspires.ftc.teamcode.Robot.robot2.Subsystems2;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class ChassisConstants2 {

    // ========== MOTOR NAMES ==========
    public static final String frName = "fr";
    public static final String flName = "fl";
    public static final String brName = "br";
    public static final String blName = "bl";

    // ========== MOTOR DIRECTIONS ==========
    public static final DcMotorSimple.Direction flDirection = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction frDirection = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction blDirection = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction brDirection = DcMotorSimple.Direction.REVERSE;

    // ========== IMU CONFIGURATION ==========
    // Adjust these based on how your Control Hub is mounted
    // Options: UP, DOWN, LEFT, RIGHT, FORWARD, BACKWARD
    public static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;

    // Options: UP, DOWN, LEFT, RIGHT, FORWARD, BACKWARD
    public static final RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    // ========== ROBOT DIMENSIONS ==========
    // Wheel diameter in inches
    public static final double WHEEL_DIAMETER_INCHES = 4.0;

    // Distance between left and right wheels (track width) in inches
    public static final double TRACK_WIDTH = 14.0;

    // Distance between front and back wheels (wheelbase) in inches
    public static final double WHEEL_BASE = 14.0;

    // ========== MOTOR SPECIFICATIONS ==========
    // For goBILDA Yellow Jacket motors
    public static final double TICKS_PER_REV = 537.7; // Adjust for your motors
    public static final double GEAR_RATIO = 1.0; // If using gear reduction

    // ========== DRIVE LIMITS ==========
    public static final double MAX_SPEED = 1.0;
    public static final double MAX_TURN_SPEED = 0.8;

    // ========== PID CONSTANTS (for autonomous) ==========
    // Translational PID
    public static final double TRANSLATIONAL_KP = 0.05;
    public static final double TRANSLATIONAL_KI = 0.0;
    public static final double TRANSLATIONAL_KD = 0.002;

    // Heading PID
    public static final double HEADING_KP = 1.0;
    public static final double HEADING_KI = 0.0;
    public static final double HEADING_KD = 0.05;

    // ========== TOLERANCES ==========
    public static final double POSITION_TOLERANCE_INCHES = 1.0;
    public static final double HEADING_TOLERANCE_DEGREES = 2.0;
}