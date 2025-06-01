package frc.robot.lib.util.interpolation;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Represents a 2D translation with interpolation support.
 */
public class InterpolatingTranslation2d extends Translation2d implements Interpolable<InterpolatingTranslation2d> {
    
    public InterpolatingTranslation2d() {
        super();
    }

    public InterpolatingTranslation2d(double x, double y) {
        super(x, y);
    }

    public InterpolatingTranslation2d(Translation2d translation) {
        super(translation.getX(), translation.getY());
    }

    /**
     * Interpolates between this translation and another translation.
     * 
     * @param other The target translation to interpolate towards.
     * @param x     Interpolation factor (0.0 = this translation, 1.0 = other translation).
     * @return The interpolated translation.
     */
    public InterpolatingTranslation2d Interpolate(InterpolatingTranslation2d other, double x) {
        if (x <= 0) {
            return new InterpolatingTranslation2d(this);
        } else if (x >= 1) {
            return new InterpolatingTranslation2d(other);
        }
        double newX = getX() + x * (other.getX() - getX());
        double newY = getY() + x * (other.getY() - getY());
        return new InterpolatingTranslation2d(newX, newY);
    }

    /**
     * Converts this translation to a rotation angle in radians.
     * 
     * @return The rotation representing the direction of this translation.
     */
    public Rotation2d getDirection() {
        return new Rotation2d(getX(), getY());
    }
}
