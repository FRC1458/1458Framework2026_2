package frc.robot.lib.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.lib.trajectory.RedTrajectory.State.ChassisAccels;

public class FieldUtil {
    public enum FieldSymmetry {
        MIRRORED,
        ROTATIONAL
    }

    // TODO: This must be tuned to the specific year's field.
    public static final FieldSymmetry CURRENT_SYMMETRY = FieldSymmetry.ROTATIONAL;

    /**
     * Flips a translation based on the current alliance.
     * @param translation The translation to flip.
     * @return The flipped translation.
     */
    public static Translation2d flipTranslation(Translation2d translation) {
        return flipTranslation(translation, CURRENT_SYMMETRY);
    }

    /**
     * Flips a rotation based on the current alliance.
     * @param rotation The rotation to flip.
     * @return The flipped rotation.
     */
    public static Rotation2d flipRotation(Rotation2d rotation) {
        return flipRotation(rotation, CURRENT_SYMMETRY);
    }

    /**
     * Flips a pose based on the current alliance.
     * @param pose The pose to flip.
     * @return The flipped pose.
     */
    public static Pose2d flipPose(Pose2d pose) {
        return flipPose(pose, CURRENT_SYMMETRY);
    }

    /**
     * Flips chassis speeds based on the current alliance.
     * @param speeds The chassis speeds to flip.
     * @return The flipped chassis speeds.
     */
    public static ChassisSpeeds flipSpeeds(ChassisSpeeds speeds) {
        return flipSpeeds(speeds, CURRENT_SYMMETRY);
    }

    /**
     * Flips chassis accelerations based on the current alliance.
     * @param accels The chassis accelerations to flip.
     * @return The flipped chassis accelerations.
     */
    public static ChassisAccels flipAccels(ChassisAccels accels) {
        return flipAccels(accels, CURRENT_SYMMETRY);
    }

    /**
     * Flips a translation according to the given field symmetry.
     * @param translation The translation to flip.
     * @param symmetry The field symmetry to use.
     * @return The flipped translation.
     */
    public static Translation2d flipTranslation(Translation2d translation, FieldSymmetry symmetry) {
        return switch (symmetry) {
            case MIRRORED -> new Translation2d(FieldLayout.FIELD_LENGTH - translation.getX(), translation.getY());
            case ROTATIONAL -> new Translation2d(FieldLayout.FIELD_LENGTH - translation.getX(), FieldLayout.FIELD_WIDTH - translation.getY());
        };
    }

    /**
     * Flips a rotation according to the given field symmetry.
     * @param rotation The rotation to flip.
     * @param symmetry The field symmetry to use.
     * @return The flipped rotation.
     */
    public static Rotation2d flipRotation(Rotation2d rotation, FieldSymmetry symmetry) {
        return switch (symmetry) {
            case MIRRORED -> new Rotation2d(-rotation.getCos(), rotation.getSin());
            case ROTATIONAL -> rotation.minus(Rotation2d.kPi);
        };
    }

    /**
     * Flips a pose according to the given field symmetry.
     * @param pose The pose to flip.
     * @param symmetry The field symmetry to use.
     * @return The flipped pose.
     */
    public static Pose2d flipPose(Pose2d pose, FieldSymmetry symmetry) {
        return new Pose2d(
            flipTranslation(pose.getTranslation(), symmetry),
            flipRotation(pose.getRotation(), symmetry)
        );
    }

    /**
     * Flips chassis speeds according to the given field symmetry.
     * @param speeds The chassis speeds to flip.
     * @param symmetry The field symmetry to use.
     * @return The flipped chassis speeds.
     */
    public static ChassisSpeeds flipSpeeds(ChassisSpeeds speeds, FieldSymmetry symmetry) {
        return switch (symmetry) {
            case MIRRORED -> new ChassisSpeeds(
                -speeds.vxMetersPerSecond,
                 speeds.vyMetersPerSecond,
                -speeds.omegaRadiansPerSecond
            );
            case ROTATIONAL -> new ChassisSpeeds(
                -speeds.vxMetersPerSecond,
                -speeds.vyMetersPerSecond,
                 speeds.omegaRadiansPerSecond
            );
        };
    }

    /**
     * Flips chassis accelerations according to the given field symmetry.
     * @param speeds The chassis accelerations to flip.
     * @param symmetry The field symmetry to use.
     * @return The flipped chassis accelerations.
     */
    public static ChassisAccels flipAccels(ChassisAccels speeds, FieldSymmetry symmetry) {
        return switch (symmetry) {
            case MIRRORED -> new ChassisAccels(
                -speeds.ax,
                 speeds.ay,
                -speeds.alpha
            );
            case ROTATIONAL -> new ChassisAccels(
                -speeds.ax,
                -speeds.ay,
                 speeds.alpha
            );
        };
    }
}
