package org.firstinspires.ftc.teamcode.Lib.STZLite.Geometry;

/**
 * A change in distance along a 2D arc since the last pose update. We can use ideas from
 * differential calculus to create new Pose2d objects from a Twist2d and vice versa.
 *
 * <p>A Twist can be used to represent a difference between two poses.
 */
public class Twist{
    /** Linear "dx" component. */
    public double dx;

    /** Linear "dy" component. */
    public double dy;

    /** Angular "dtheta" component (radians). */
    public double dtheta;

    /** Default constructor. */
    public Twist() {}

    /**
     * Constructs a Twist2d with the given values.
     *
     * @param dx Change in x direction relative to robot.
     * @param dy Change in y direction relative to robot.
     * @param dtheta Change in angle relative to robot.
     */
    public Twist(double dx, double dy, double dtheta) {
        this.dx = dx;
        this.dy = dy;
        this.dtheta = dtheta;
    }

}
