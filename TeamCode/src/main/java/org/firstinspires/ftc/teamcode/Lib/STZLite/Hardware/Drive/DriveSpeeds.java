package org.firstinspires.ftc.teamcode.Lib.STZLite.Hardware.Drive;

import org.firstinspires.ftc.teamcode.Lib.STZLite.Geometry.Pose;
import org.firstinspires.ftc.teamcode.Lib.STZLite.Geometry.Rotation;
import org.firstinspires.ftc.teamcode.Lib.STZLite.Geometry.Translation;
import org.firstinspires.ftc.teamcode.Lib.STZLite.Geometry.Twist;

public class DriveSpeeds {
    /** Velocity along the x-axis. (Fwd is +) */
    public double vxMetersPerSecond;

    /** Velocity along the y-axis. (Left is +) */
    public double vyMetersPerSecond;

    /** Represents the angular velocity of the robot frame. (CCW is +) */
    public double omegaRadiansPerSecond;

    /**
     * Constructs a ChassisSpeeds object.
     *
     * @param vxMetersPerSecond Forward velocity.
     * @param vyMetersPerSecond Sideways velocity.
     * @param omegaRadiansPerSecond Angular velocity.
     */
    public DriveSpeeds(
            double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
        this.vxMetersPerSecond = vxMetersPerSecond;
        this.vyMetersPerSecond = vyMetersPerSecond;
        this.omegaRadiansPerSecond = omegaRadiansPerSecond;
    }

    /**
     * Creates a Twist2d from ChassisSpeeds.
     *
     * @param dtSeconds The duration of the timestep.
     * @return Twist2d.
     */
    public Twist toTwist(double dtSeconds) {
        return new Twist(
                vxMetersPerSecond * dtSeconds,
                vyMetersPerSecond * dtSeconds,
                omegaRadiansPerSecond * dtSeconds);
    }

    /**
     * Discretizes a continuous-time chassis speed.
     *
     * <p>This function converts a continuous-time chassis speed into a discrete-time one such that
     * when the discrete-time chassis speed is applied for one timestep, the robot moves as if the
     * velocity components are independent (i.e., the robot moves v_x * dt along the x-axis, v_y * dt
     * along the y-axis, and omega * dt around the z-axis).
     *
     * <p>This is useful for compensating for translational skew when translating and rotating a
     * holonomic (swerve or mecanum) drivetrain. However, scaling down the ChassisSpeeds after
     * discretizing (e.g., when desaturating swerve module speeds) rotates the direction of net motion
     * in the opposite direction of rotational velocity, introducing a different translational skew
     * which is not accounted for by discretization.
     *
     * @param vxMetersPerSecond Forward velocity.
     * @param vyMetersPerSecond Sideways velocity.
     * @param omegaRadiansPerSecond Angular velocity.
     * @param dtSeconds The duration of the timestep the speeds should be applied for.
     * @return Discretized ChassisSpeeds.
     */
    public static DriveSpeeds discretize(
            double vxMetersPerSecond,
            double vyMetersPerSecond,
            double omegaRadiansPerSecond,
            double dtSeconds) {
        // Construct the desired pose after a timestep, relative to the current pose. The desired pose
        // has decoupled translation and rotation.
        Pose desiredDeltaPose =
                new Pose(
                        vxMetersPerSecond * dtSeconds,
                        vyMetersPerSecond * dtSeconds,
                        new Rotation(omegaRadiansPerSecond * dtSeconds));

        // Find the chassis translation/rotation deltas in the robot frame that move the robot from its
        // current pose to the desired pose
        Twist twist = Pose.kZero.log(desiredDeltaPose);

        // Turn the chassis translation/rotation deltas into average velocities
        return new DriveSpeeds(twist.dx / dtSeconds, twist.dy / dtSeconds, twist.dtheta / dtSeconds);
    }


    /**
     * Converts a user provided field-relative set of speeds into a robot-relative ChassisSpeeds
     * object.
     *
     * @param vxMetersPerSecond The component of speed in the x direction relative to the field.
     *     Positive x is away from your alliance wall.
     * @param vyMetersPerSecond The component of speed in the y direction relative to the field.
     *     Positive y is to your left when standing behind the alliance wall.
     * @param omegaRadiansPerSecond The angular rate of the robot.
     * @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is
     *     considered to be zero when it is facing directly away from your alliance station wall.
     *     Remember that this should be CCW positive.
     * @return ChassisSpeeds object representing the speeds in the robot's frame of reference.
     */
    public static DriveSpeeds fromFieldRelativeSpeeds(
            double vxMetersPerSecond,
            double vyMetersPerSecond,
            double omegaRadiansPerSecond,
            Rotation robotAngle) {
        // CW rotation into chassis frame
        Translation rotated =
                new Translation(vxMetersPerSecond, vyMetersPerSecond).rotateBy(robotAngle.unaryMinus());
        return new DriveSpeeds(rotated.getX(), rotated.getY(), omegaRadiansPerSecond);
    }


    /**
     * Converts a user provided robot-relative set of speeds into a field-relative ChassisSpeeds
     * object.
     *
     * @param vxMetersPerSecond The component of speed in the x direction relative to the robot.
     *     Positive x is towards the robot's front.
     * @param vyMetersPerSecond The component of speed in the y direction relative to the robot.
     *     Positive y is towards the robot's left.
     * @param omegaRadiansPerSecond The angular rate of the robot.
     * @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is
     *     considered to be zero when it is facing directly away from your alliance station wall.
     *     Remember that this should be CCW positive.
     * @return ChassisSpeeds object representing the speeds in the field's frame of reference.
     */
    public static DriveSpeeds fromRobotRelativeSpeeds(
            double vxMetersPerSecond,
            double vyMetersPerSecond,
            double omegaRadiansPerSecond,
            Rotation robotAngle) {
        // CCW rotation out of chassis frame
        Translation rotated = new Translation(vxMetersPerSecond, vyMetersPerSecond).rotateBy(robotAngle);
        return new DriveSpeeds(rotated.getX(), rotated.getY(), omegaRadiansPerSecond);
    }

    /**
     * Converts a user provided robot-relative ChassisSpeeds object into a field-relative
     * ChassisSpeeds object.
     *
     * @param robotRelativeSpeeds The ChassisSpeeds object representing the speeds in the robot frame
     *     of reference. Positive x is towards the robot's front. Positive y is towards the robot's
     *     left.
     * @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is
     *     considered to be zero when it is facing directly away from your alliance station wall.
     *     Remember that this should be CCW positive.
     * @return ChassisSpeeds object representing the speeds in the field's frame of reference.
     */
    public static DriveSpeeds fromRobotRelativeSpeeds(
            DriveSpeeds robotRelativeSpeeds, Rotation robotAngle) {
        return fromRobotRelativeSpeeds(
                robotRelativeSpeeds.vxMetersPerSecond,
                robotRelativeSpeeds.vyMetersPerSecond,
                robotRelativeSpeeds.omegaRadiansPerSecond,
                robotAngle);
    }

    /**
     * Adds two ChassisSpeeds and returns the sum.
     *
     * <p>For example, ChassisSpeeds{1.0, 0.5, 0.75} + ChassisSpeeds{2.0, 1.5, 0.25} =
     * ChassisSpeeds{3.0, 2.0, 1.0}
     *
     * @param other The ChassisSpeeds to add.
     * @return The sum of the ChassisSpeeds.
     */
    public DriveSpeeds plus(DriveSpeeds other) {
        return new DriveSpeeds(
                vxMetersPerSecond + other.vxMetersPerSecond,
                vyMetersPerSecond + other.vyMetersPerSecond,
                omegaRadiansPerSecond + other.omegaRadiansPerSecond);
    }

    /**
     * Subtracts the other ChassisSpeeds from the current ChassisSpeeds and returns the difference.
     *
     * <p>For example, ChassisSpeeds{5.0, 4.0, 2.0} - ChassisSpeeds{1.0, 2.0, 1.0} =
     * ChassisSpeeds{4.0, 2.0, 1.0}
     *
     * @param other The ChassisSpeeds to subtract.
     * @return The difference between the two ChassisSpeeds.
     */
    public DriveSpeeds minus(DriveSpeeds other) {
        return new DriveSpeeds(
                vxMetersPerSecond - other.vxMetersPerSecond,
                vyMetersPerSecond - other.vyMetersPerSecond,
                omegaRadiansPerSecond - other.omegaRadiansPerSecond);
    }

    /**
     * Returns the inverse of the current ChassisSpeeds. This is equivalent to negating all components
     * of the ChassisSpeeds.
     *
     * @return The inverse of the current ChassisSpeeds.
     */
    public DriveSpeeds unaryMinus() {
        return new DriveSpeeds(-vxMetersPerSecond, -vyMetersPerSecond, -omegaRadiansPerSecond);
    }

    /**
     * Multiplies the ChassisSpeeds by a scalar and returns the new ChassisSpeeds.
     *
     * <p>For example, ChassisSpeeds{2.0, 2.5, 1.0} * 2 = ChassisSpeeds{4.0, 5.0, 1.0}
     *
     * @param scalar The scalar to multiply by.
     * @return The scaled ChassisSpeeds.
     */
    public DriveSpeeds times(double scalar) {
        return new DriveSpeeds(
                vxMetersPerSecond * scalar, vyMetersPerSecond * scalar, omegaRadiansPerSecond * scalar);
    }

    /**
     * Divides the ChassisSpeeds by a scalar and returns the new ChassisSpeeds.
     *
     * <p>For example, ChassisSpeeds{2.0, 2.5, 1.0} / 2 = ChassisSpeeds{1.0, 1.25, 0.5}
     *
     * @param scalar The scalar to divide by.
     * @return The scaled ChassisSpeeds.
     */
    public DriveSpeeds div(double scalar) {
        return new DriveSpeeds(
                vxMetersPerSecond / scalar, vyMetersPerSecond / scalar, omegaRadiansPerSecond / scalar);
    }



}
