package org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive;

public class VisionConstants {
    public static  final  String limelightName = "limelight";
    public static  final  double STALENESS_THRESHOLD_MS = 100.0;

    // --- MEASURE THESE (IN INCHES) ---
    // Height of the center of the AprilTag from the floor
    public static final double TARGET_HEIGHT = 13.5;

    // Height of your Limelight lens from the floor
    public static final double CAMERA_HEIGHT = 9.5;

    // Angle of your camera (Degrees). 0 = Straight forward, 20 = Tilted up
    public static final double CAMERA_ANGLE = 15.0;

    // --- SHOOTING PHYSICS (TUNE THESE) ---
    // Example: At 0 inches, shoot 2000 RPM. For every inch away, add 50 RPM.
    public static final double BASE_RPM = 2500.0;
    public static final double RPM_PER_INCH = 50.0;

}
