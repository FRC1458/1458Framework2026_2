package frc.robot.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;

/**
 * Contains basic functions that are used often.
 */
public class Util {
	public static final double kEpsilon = 1e-12;

	/**
	 * Prevent this class from being instantiated.
	 */
	private Util() {}

	public static class MathUtil {
		/**
		 * Limits the given input to the given magnitude.
		 */
		public static double limit(double v, double maxMagnitude) {
			return limit(v, -maxMagnitude, maxMagnitude);
		}

		public static double limit(double v, double min, double max) {
			return Math.min(max, Math.max(min, v));
		}

		public static boolean inRange(double v, double maxMagnitude) {
			return inRange(v, -maxMagnitude, maxMagnitude);
		}

		/**
		 * Checks if the given input is within the range (min, max), both exclusive.
		 */
		public static boolean inRange(double v, double min, double max) {
			return v > min && v < max;
		}

		public static double interpolate(double a, double b, double x) {
			x = limit(x, 0.0, 1.0);
			return a + (b - a) * x;
		}

		public static String joinStrings(final String delim, final List<?> strings) {
			StringBuilder sb = new StringBuilder();
			for (int i = 0; i < strings.size(); ++i) {
				sb.append(strings.get(i).toString());
				if (i < strings.size() - 1) {
					sb.append(delim);
				}
			}
			return sb.toString();
		}

		public static boolean epsilonEquals(double a, double b, double epsilon) {
			return (a - epsilon <= b) && (a + epsilon >= b);
		}

		public static boolean epsilonEquals(double a, double b) {
			return epsilonEquals(a, b, kEpsilon);
		}

		public static boolean epsilonEquals(int a, int b, int epsilon) {
			return (a - epsilon <= b) && (a + epsilon >= b);
		}

		public static boolean allCloseTo(final List<Double> list, double value, double epsilon) {
			boolean result = true;
			for (Double value_in : list) {
				result &= epsilonEquals(value_in, value, epsilon);
			}
			return result;
		}

		public static double clamp(double value, double min, double max) {
			if (min > max) {
				throw new IllegalArgumentException("min must not be greater than max");
			}

			return Math.max(min, Math.min(value, max));
		}
	}
	public static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
		double lowerBound;
		double upperBound;
		double lowerOffset = scopeReference % 360;
		if (lowerOffset >= 0) {
			lowerBound = scopeReference - lowerOffset;
			upperBound = scopeReference + (360 - lowerOffset);
		} else {
			upperBound = scopeReference - lowerOffset;
			lowerBound = scopeReference - (360 + lowerOffset);
		}
		while (newAngle < lowerBound) {
			newAngle += 360;
		}
		while (newAngle > upperBound) {
			newAngle -= 360;
		}
		if (newAngle - scopeReference > 180) {
			newAngle -= 360;
		} else if (newAngle - scopeReference < -180) {
			newAngle += 360;
		}
		return newAngle;
	}

	/**
	 * Applies a deadband to a specified value
	 * @param val the value to deadband
	 * @param deadband the deadband threshold
	 * @return the deadbanded value
	 */
	public static double deadBand(double val, double deadband) {
		return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
	}

	public static double scaledDeadband(double value, double maxValue, double deadband) {
		double deadbandedValue = deadBand(value, deadband);
		if (MathUtil.epsilonEquals(deadbandedValue, 0.0)) return 0.0;
		return Math.signum(deadbandedValue) * ((Math.abs(deadbandedValue) - deadband) / (maxValue - deadband));
	}

	public static Rotation2d robotToFieldRelative(Rotation2d rot, boolean is_red_alliance) {
		if (is_red_alliance) {
			return rot.rotateBy(Rotation2d.fromDegrees(180.0));
		} else {
			return rot;
		}
	}

	public static double boundAngleNeg180to180Degrees(double angle) {
		// Naive algorithm
		while (angle >= 180.0) {
			angle -= 360.0;
		}
		while (angle < -180.0) {
			angle += 360.0;
		}
		return angle;
	}

	public static double boundAngle0to360Degrees(double angle) {
		// Naive algorithm
		while (angle >= 360.0) {
			angle -= 360.0;
		}
		while (angle < 0.0) {
			angle += 360.0;
		}
		return angle;
	}

	public static double boundToScope(double scopeFloor, double scopeCeiling, double argument) {
		double stepSize = scopeCeiling - scopeFloor;
		while (argument >= scopeCeiling) {
			argument -= stepSize;
		}
		while (argument < scopeFloor) {
			argument += stepSize;
		}
		return argument;
	}

	public static Pose2d inversePose2d(Pose2d curPose) {
		// Invert the rotation
		Rotation2d inverseRotation = curPose.getRotation().unaryMinus();
		
		// Invert the translation by applying the inverse rotation to the negative translation
		Translation2d inverseTranslation = curPose.getTranslation().unaryMinus().rotateBy(inverseRotation);
		
		// Return a new Pose2d with the inverted translation and rotation
		return new Pose2d(inverseTranslation, inverseRotation);
	}

	/**
     * Obtain a new Pose2d from a curvature velocity. See:
     * https://github.com/strasdat/Sophus/blob/master/sophus/se2.hpp
     */
	private final static double kEps = 1E-9;	//only used by exp() and log() mapping functions
    public static Pose2d expMap(final Twist2d delta) {
        double sin_theta = Math.sin(delta.dtheta);
        double cos_theta = Math.cos(delta.dtheta);
        double s, c;
        if (Math.abs(delta.dtheta) < kEps) {
            s = 1.0 - 1.0 / 6.0 * delta.dtheta * delta.dtheta;
            c = .5 * delta.dtheta;
        } else {
            s = sin_theta / delta.dtheta;
            c = (1.0 - cos_theta) / delta.dtheta;
        }
        return new Pose2d(new Translation2d(delta.dx * s - delta.dy * c, delta.dx * c + delta.dy * s),
                new Rotation2d(cos_theta, sin_theta));
    }
	
    /**
     * Logical inverse of the exp() function above.
     */
    public static Twist2d logMap(final Pose2d transform) {
        final double dtheta = transform.getRotation().getRadians();
        final double half_dtheta = 0.5 * dtheta;
        final double cos_minus_one = transform.getRotation().getCos() - 1.0;
        double halftheta_by_tan_of_halfdtheta;
        if (Math.abs(cos_minus_one) < kEps) {
            halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        } else {
            halftheta_by_tan_of_halfdtheta = -(half_dtheta * transform.getRotation().getSin()) / cos_minus_one;
        }
        final Translation2d translation_part = transform.getTranslation()
                .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta)); //take out the normalize: false parameter from citrus code
        return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
    }

	// ported from citrus Twist2d.scaled() method
	public static Twist2d scaledTwist2d(Twist2d twist, double scale) {
        return new Twist2d(twist.dx * scale, twist.dy * scale, twist.dtheta * scale);
    }

	//compare delta of two chassisspeeds is less than epsilon
	public static boolean chassisSpeedsEpsilonEquals(ChassisSpeeds speed1, ChassisSpeeds other, double epsilon) {
		return MathUtil.epsilonEquals(speed1.vxMetersPerSecond, other.vxMetersPerSecond, epsilon)
				&& MathUtil.epsilonEquals(speed1.vyMetersPerSecond, other.vyMetersPerSecond, epsilon)
				&& MathUtil.epsilonEquals(speed1.omegaRadiansPerSecond, other.omegaRadiansPerSecond, epsilon);
	}


	public static Translation2d translateBy(Translation2d a, Translation2d b){
		return new Translation2d(a.getX()+b.getX(),a.getY()+b.getY());
	}
	public static Pose2d translateBy(Pose2d a, Pose2d b){
		return new Pose2d(translateBy(a.getTranslation(), b.getTranslation()),a.getRotation().plus(b.getRotation()));
	}

	public static double twist2dMagnitude(Twist2d t) {
		return Math.sqrt(t.dx * t.dx + t.dy * t.dy);
	}
}
