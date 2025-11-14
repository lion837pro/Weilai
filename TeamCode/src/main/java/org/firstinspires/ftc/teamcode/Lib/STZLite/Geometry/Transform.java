package org.firstinspires.ftc.teamcode.Lib.STZLite.Geometry;

/** Represents a transformation for a Pose2d in the pose's frame. */
public class Transform{
    /**
     * A preallocated Transform2d representing no transformation.
     *
     * <p>This exists to avoid allocations for common transformations.
     */
    public static final Transform kZero = new Transform();

    private final Translation m_translation;
    private final Rotation m_rotation;

    /**
     * Constructs the transform that maps the initial pose to the final pose.
     *
     * @param initial The initial pose for the transformation.
     * @param last The final pose for the transformation.
     */
    public Transform(Pose initial, Pose last) {
        // We are rotating the difference between the translations
        // using a clockwise rotation matrix. This transforms the global
        // delta into a local delta (relative to the initial pose).
        m_translation =
                last.getTranslation()
                        .minus(initial.getTranslation())
                        .rotateBy(initial.getRotation().unaryMinus());

        m_rotation = last.getRotation().minus(initial.getRotation());
    }

    /**
     * Constructs a transform with the given translation and rotation components.
     *
     * @param translation Translational component of the transform.
     * @param rotation Rotational component of the transform.
     */
    public Transform(Translation translation, Rotation rotation) {
        m_translation = translation;
        m_rotation = rotation;
    }

    /**
     * Constructs a transform with x and y translations instead of a separate Translation2d.
     *
     * @param x The x component of the translational component of the transform.
     * @param y The y component of the translational component of the transform.
     * @param rotation The rotational component of the transform.
     */
    public Transform(double x, double y, Rotation rotation) {
        m_translation = new Translation(x, y);
        m_rotation = rotation;
    }

    /** Constructs the identity transform -- maps an initial pose to itself. */
    public Transform() {
        m_translation = Translation.kZero;
        m_rotation = Rotation.kZero;
    }

    /**
     * Multiplies the transform by the scalar.
     *
     * @param scalar The scalar.
     * @return The scaled Transform2d.
     */
    public Transform times(double scalar) {
        return new Transform(m_translation.times(scalar), m_rotation.times(scalar));
    }

    /**
     * Divides the transform by the scalar.
     *
     * @param scalar The scalar.
     * @return The scaled Transform2d.
     */
    public Transform div(double scalar) {
        return times(1.0 / scalar);
    }

    /**
     * Composes two transformations. The second transform is applied relative to the orientation of
     * the first.
     *
     * @param other The transform to compose with this one.
     * @return The composition of the two transformations.
     */
    public Transform plus(Transform other) {
        return new Transform(Pose.kZero, Pose.kZero.transformBy(this).transformBy(other));
    }

    /**
     * Returns the translation component of the transformation.
     *
     * @return The translational component of the transform.
     */
    public Translation getTranslation() {
        return m_translation;
    }

    /**
     * Returns the X component of the transformation's translation.
     *
     * @return The x component of the transformation's translation.
     */
    public double getX() {
        return m_translation.getX();
    }

    /**
     * Returns the Y component of the transformation's translation.
     *
     * @return The y component of the transformation's translation.
     */
    public double getY() {
        return m_translation.getY();
    }


    /**
     * Returns the rotational component of the transformation.
     *
     * @return Reference to the rotational component of the transform.
     */
    public Rotation getRotation() {
        return m_rotation;
    }

    /**
     * Invert the transformation. This is useful for undoing a transformation.
     *
     * @return The inverted transformation.
     */
    public Transform inverse() {
        // We are rotating the difference between the translations
        // using a clockwise rotation matrix. This transforms the global
        // delta into a local delta (relative to the initial pose).
        return new Transform(
                getTranslation().unaryMinus().rotateBy(getRotation().unaryMinus()),
                getRotation().unaryMinus());
    }

    /**
     * Returns an array representation of the transform.
     *
     * @return An array representation of the transform.
     */
    public double[] toArray() {
        return new double[] {getX(), getY(), getRotation().getRadians()};
    }

}

