package frc.robot.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.Interpolator;

public class InterpolationUtil {
    public static Interpolator<Pose2d> poseInterpolator =
        new Interpolator<>() {
            @Override
            public Pose2d interpolate(Pose2d startValue, Pose2d endValue, double t) {
                if (t <= 0) {
                    return startValue;
                } else if (t >= 1) {
                    return endValue;
                }
                
                // Calculate the difference in translation and rotation (as a pose)
                Pose2d relativePose = endValue.relativeTo(startValue);

                // Convert this relative pose to a Twist2d for interpolation
                Twist2d twist = new Twist2d(
                    relativePose.getX(), 
                    relativePose.getY(), 
                    relativePose.getRotation().getRadians()
                );
                
                // Scale the twist by the interpolation factor
                Twist2d scaledTwist = new Twist2d(
                    twist.dx * t,
                    twist.dy * t,
                    twist.dtheta * t
                );
                
                // Apply the scaled twist to this pose and return the result
                return startValue.exp(scaledTwist);
            };
        };

    public static Interpolator<Translation2d> translationInterpolator =
        new Interpolator<>() {
            @Override
            public Translation2d interpolate(Translation2d startValue, Translation2d endValue, double t) {
                if (t <= 0) {
                    return startValue;
                } else if (t >= 1) {
                    return endValue;
                }
                double newX = startValue.getX() + t * (endValue.getX() - startValue.getX());
                double newY = startValue.getY() + t * (endValue.getY() - startValue.getY());
                return new Translation2d(newX, newY);
            };
        };
}
