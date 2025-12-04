package org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive;

public class VisionConstants {
    public static final String limelightName = "limelight";
    public static final double STALENESS_THRESHOLD_MS = 250;

    // --- CAMERA MOUNTING (IN INCHES) ---
    public static final double TARGET_HEIGHT = 13.5;   // AprilTag height from floor
    public static final double CAMERA_HEIGHT = 9.5;    // Limelight height from floor
    public static final double CAMERA_ANGLE = 30;      // Camera tilt angle (degrees)

    // --- SHOOTING PHYSICS (TUNE THESE) ---
    public static final double BASE_RPM = 1400.0;
    public static final double RPM_PER_INCH = 15.0;

    // ===== APRILTAG IDS FOR ALIGNMENT =====
    // Only these tags will trigger auto-alignment
    public static final int ALIGNMENT_TAG_1 = 20;
    public static final int ALIGNMENT_TAG_2 = 24;

    /**
     * Check if a tag ID is valid for auto-alignment
     */
    public static boolean isAlignmentTag(int tagId) {
        return tagId == ALIGNMENT_TAG_1 || tagId == ALIGNMENT_TAG_2;
    }

    // ===== APRILTAG IDS FOR COLOR SORTING =====
    // These tags indicate the goal pattern for shooting
    // Pattern: slot 0, slot 1, slot 2 (G = Green, P = Purple)
    public static final int COLOR_SORT_TAG_GPP = 21;  // Green, Purple, Purple
    public static final int COLOR_SORT_TAG_PGP = 22;  // Purple, Green, Purple
    public static final int COLOR_SORT_TAG_PPG = 23;  // Purple, Purple, Green

    /**
     * Check if a tag ID is a color sorting tag
     */
    public static boolean isColorSortTag(int tagId) {
        return tagId == COLOR_SORT_TAG_GPP ||
               tagId == COLOR_SORT_TAG_PGP ||
               tagId == COLOR_SORT_TAG_PPG;
    }

    /**
     * Get the target color pattern for a given tag ID.
     * Returns array of 3 BallColors: [slot0, slot1, slot2]
     * Returns null if not a color sort tag.
     */
    public static BallColor[] getTargetPattern(int tagId) {
        switch (tagId) {
            case COLOR_SORT_TAG_GPP:  // ID 21: Green, Purple, Purple
                return new BallColor[]{BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE};
            case COLOR_SORT_TAG_PGP:  // ID 22: Purple, Green, Purple
                return new BallColor[]{BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE};
            case COLOR_SORT_TAG_PPG:  // ID 23: Purple, Purple, Green
                return new BallColor[]{BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};
            default:
                return null;
        }
    }

    /**
     * Get the slot index that should have a GREEN ball for a given tag.
     * Returns -1 if tag is not a color sort tag.
     */
    public static int getGreenSlotForTag(int tagId) {
        switch (tagId) {
            case COLOR_SORT_TAG_GPP: return 0;  // ID 21: Green at slot 0
            case COLOR_SORT_TAG_PGP: return 1;  // ID 22: Green at slot 1
            case COLOR_SORT_TAG_PPG: return 2;  // ID 23: Green at slot 2
            default: return -1;
        }
    }

    // ===== BALL COLOR ENUM =====
    public enum BallColor {
        GREEN,
        PURPLE,
        UNKNOWN
    }
}
