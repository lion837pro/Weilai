package org.firstinspires.ftc.teamcode.Lib.STZLite.Geometry;

import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class Translation {
    /**
     * A preallocated Translation2d representing the origin.
     *
     * <p>This exists to avoid allocations for common translations.
     */
    public static final Translation kZero = new Translation();

    private double m_x;
    private double m_y;

    /** Constructs a Translation2d with X and Y components equal to zero. */
    public Translation() {
        this(0.0, 0.0);
    }

    /**
     * Constructs a Translation2d with the X and Y components equal to the provided values.
     *
     * @param x The x component of the translation.
     * @param y The y component of the translation.
     */
    public Translation(double x, double y) {
        m_x = x;
        m_y = y;
    }

    /**
     * Constructs a Translation2d with the provided distance and angle. This is essentially converting
     * from polar coordinates to Cartesian coordinates.
     *
     * @param distance The distance from the origin to the end of the translation.
     * @param angle The angle between the x-axis and the translation vector.
     */
    public Translation(double distance, Rotation angle) {
        m_x = distance * angle.getCos();
        m_y = distance * angle.getSin();
    }

    public void setX(double x) {
        this.m_x = x;
    }

    public void setY(double y) {
        this.m_y = y;
    }

    /**

    /**
     * Calculates the distance between two translations in 2D space.
     *
     * <p>The distance between translations is defined as √((x₂−x₁)²+(y₂−y₁)²).
     *
     * @param other The translation to compute the distance to.
     * @return The distance between the two translations.
     */
    public double getDistance(Translation other) {
        return Math.hypot(other.m_x - m_x, other.m_y - m_y);
    }

    /**
     * Returns the X component of the translation.
     *
     * @return The X component of the translation.
     */
    public double getX() {
        return m_x;
    }

    /**
     * Returns the Y component of the translation.
     *
     * @return The Y component of the translation.
     */
    public double getY() {
        return m_y;
    }

    /**
     * Returns the norm, or distance from the origin to the translation.
     *
     * @return The norm of the translation.
     */
    public double getNorm() {
        return Math.hypot(m_x, m_y);
    }

    /**
     * Returns the angle this translation forms with the positive X axis.
     *
     * @return The angle of the translation
     */
    public Rotation getAngle() {
        return new Rotation(m_x, m_y);
    }

    /**
     * Applies a rotation to the translation in 2D space.
     *
     * <p>This multiplies the translation vector by a counterclockwise rotation matrix of the given
     * angle.
     *
     * <pre>
     * [x_new]   [other.cos, -other.sin][x]
     * [y_new] = [other.sin,  other.cos][y]
     * </pre>
     *
     * <p>For example, rotating a Translation2d of &lt;2, 0&gt; by 90 degrees will return a
     * Translation2d of &lt;0, 2&gt;.
     *
     * @param other The rotation to rotate the translation by.
     * @return The new rotated translation.
     */
    public Translation rotateBy(Rotation other) {
        return new Translation(
                m_x * other.getCos() - m_y * other.getSin(), m_x * other.getSin() + m_y * other.getCos());
    }

    /**
     * Rotates this translation around another translation in 2D space.
     *
     * <pre>
     * [x_new]   [rot.cos, -rot.sin][x - other.x]   [other.x]
     * [y_new] = [rot.sin,  rot.cos][y - other.y] + [other.y]
     * </pre>
     *
     * @param other The other translation to rotate around.
     * @param rot The rotation to rotate the translation by.
     * @return The new rotated translation.
     */
    public Translation rotateAround(Translation other, Rotation rot) {
        return new Translation(
                (m_x - other.getX()) * rot.getCos() - (m_y - other.getY()) * rot.getSin() + other.getX(),
                (m_x - other.getX()) * rot.getSin() + (m_y - other.getY()) * rot.getCos() + other.getY());
    }

    /**
     * Returns the sum of two translations in 2D space.
     *
     * <p>For example, Translation3d(1.0, 2.5) + Translation3d(2.0, 5.5) = Translation3d{3.0, 8.0).
     *
     * @param other The translation to add.
     * @return The sum of the translations.
     */
    public Translation plus(Translation other) {
        return new Translation(m_x + other.m_x, m_y + other.m_y);
    }

    /**
     * Returns the difference between two translations.
     *
     * <p>For example, Translation2d(5.0, 4.0) - Translation2d(1.0, 2.0) = Translation2d(4.0, 2.0).
     *
     * @param other The translation to subtract.
     * @return The difference between the two translations.
     */
    public Translation minus(Translation other) {
        return new Translation(m_x - other.m_x, m_y - other.m_y);
    }

    /**
     * Returns the inverse of the current translation. This is equivalent to rotating by 180 degrees,
     * flipping the point over both axes, or negating all components of the translation.
     *
     * @return The inverse of the current translation.
     */
    public Translation unaryMinus() {
        return new Translation(-m_x, -m_y);
    }

    /**
     * Returns the translation multiplied by a scalar.
     *
     * <p>For example, Translation2d(2.0, 2.5) * 2 = Translation2d(4.0, 5.0).
     *
     * @param scalar The scalar to multiply by.
     * @return The scaled translation.
     */
    public Translation times(double scalar) {
        return new Translation(m_x * scalar, m_y * scalar);
    }

    /**
     * Returns the translation divided by a scalar.
     *
     * <p>For example, Translation3d(2.0, 2.5) / 2 = Translation3d(1.0, 1.25).
     *
     * @param scalar The scalar to multiply by.
     * @return The reference to the new mutated object.
     */
    public Translation div(double scalar) {
        return new Translation(m_x / scalar, m_y / scalar);
    }

    /**
     * Returns the nearest Translation2d from a list of translations.
     *
     * @param translations The list of translations.
     * @return The nearest Translation2d from the list.
     */
    public Translation nearest(List<Translation> translations) {
        return Collections.min(translations, Comparator.comparing(this::getDistance));
    }

    /**
     * Returns the translation as an array.
     *
     * @return The translation as an array.
     */
    public double[] toArray() {
        return new double[] {m_x, m_y};
    }


}