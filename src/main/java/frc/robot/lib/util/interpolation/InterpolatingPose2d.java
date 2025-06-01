package frc.robot.lib.util.interpolation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

/**
 * Represents a 2d pose (rigid transform) containing translational and rotational elements.
 * <p>
 * Inspired by Sophus (https://github.com/strasdat/Sophus/tree/master/sophus)
 */
public class InterpolatingPose2d extends Pose2d implements Interpolable<InterpolatingPose2d>{
    public InterpolatingPose2d() {
        super();
    }

    public InterpolatingPose2d(double x, double y, Rotation2d rotation) {
        super(x, y, rotation);
    }

    public InterpolatingPose2d(final Translation2d translation, final Rotation2d rotation) {
        super(translation, rotation);
    }

    public InterpolatingPose2d(final Pose2d other) {
        super(other.getTranslation(), other.getRotation());
    }

    /**
     * Performs an interpolation between this pose and another pose.
     * @param other The pose to interpolate towards.
     * @param x The interpolation factor (0.0 = this pose, 1.0 = other pose).
     * @return The interpolated pose.
     */
    public InterpolatingPose2d Interpolate(final InterpolatingPose2d other, double x) {
        if (x <= 0) {
            return new InterpolatingPose2d(this);
        } else if (x >= 1) {
            return new InterpolatingPose2d(other);
        }
        
        // Calculate the difference in translation and rotation (as a pose)
        Pose2d relativePose = other.relativeTo(this);

        // Convert this relative pose to a Twist2d for interpolation
        Twist2d twist = new Twist2d(
            relativePose.getX(), 
            relativePose.getY(), 
            relativePose.getRotation().getRadians()
        );
        
        // Scale the twist by the interpolation factor
        Twist2d scaledTwist = new Twist2d(
            twist.dx * x,
            twist.dy * x,
            twist.dtheta * x
        );
        
        // Apply the scaled twist to this pose and return the result
        return new InterpolatingPose2d(this.exp(scaledTwist));
    }
}
