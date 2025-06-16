package frc.robot.lib.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionPoseAcceptor {
	private static final double kFieldBorderMargin = 0.5;
	private static final double kMaxVisionCorrection = 2.0; // Jump from fused pose

	Pose2d mLastVisionFieldToVehicle = null;

	public static Twist2d log(final Pose2d transform) {
		final double dtheta = transform.getRotation().getRadians();
		final double half_dtheta = 0.5 * dtheta;
		final double cos_minus_one = transform.getRotation().getCos() - 1.0;
		double halftheta_by_tan_of_halfdtheta;
		if (Math.abs(cos_minus_one) < 1E-9) {
			halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
		} else {
			halftheta_by_tan_of_halfdtheta = -(half_dtheta * transform.getRotation().getSin()) / cos_minus_one;
		}
		final Translation2d translation_part = transform.getTranslation()
				.rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
		return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
	}

	public Pose2d inverse(Translation2d translation, Rotation2d rotation) {
		Rotation2d rotation_inverted = Rotation2d.fromRadians(-rotation.getRadians());
		return new Pose2d(new Translation2d(-translation.getX(), -translation.getY()).rotateBy(rotation_inverted),
				rotation_inverted);
	}

	public boolean shouldAcceptVision(
			double timestamp,
			Pose2d visionFieldToVehicle,
			Pose2d lastFieldToVehicle,
			Twist2d robotVelocity,
			boolean isInAuto) {

		// If first update, trust
		if (mLastVisionFieldToVehicle == null) {
			mLastVisionFieldToVehicle = visionFieldToVehicle;
			return true;
		}

		// Write last pose early because we return out of the method
		mLastVisionFieldToVehicle = visionFieldToVehicle;

		// Check out of field
		if (visionFieldToVehicle.getTranslation().getX() < -kFieldBorderMargin
				|| visionFieldToVehicle.getTranslation().getX() > FieldLayout.FIELD_LENGTH + kFieldBorderMargin
				|| visionFieldToVehicle.getTranslation().getY() < -kFieldBorderMargin
				|| visionFieldToVehicle.getTranslation().getY() > FieldLayout.FIELD_WIDTH + kFieldBorderMargin) {
			SmartDashboard.putString("Vision validation", "Outside field");
			return false;
		}

		if ((robotVelocity.dy == 0
				? Math.abs(robotVelocity.dx)
				: Math.hypot(robotVelocity.dx, robotVelocity.dy)) > 4.0) {
			SmartDashboard.putString("Vision validation", "Max velocity");
			return false;
		}

		if (isInAuto) {
			// Check max correction
			final Pose2d inverse = inverse(visionFieldToVehicle.getTranslation(), visionFieldToVehicle.getRotation());
			final Twist2d logTwist = log(new Pose2d(
					inverse.getTranslation().plus(lastFieldToVehicle.getTranslation().rotateBy(inverse.getRotation())),
					inverse.getRotation().rotateBy(lastFieldToVehicle.getRotation())));
			if ((logTwist.dy == 0
					? Math.abs(logTwist.dx)
					: Math.hypot(logTwist.dx, logTwist.dy)) > kMaxVisionCorrection) {
				SmartDashboard.putString("Vision validation", "Max correction");
				return false;
			}
		}

		SmartDashboard.putString("Vision validation", "OK");
		return true;
	}
}
