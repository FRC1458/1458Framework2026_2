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

    public static final FieldSymmetry CURRENT_SYMMETRY = FieldSymmetry.ROTATIONAL;

    public static Translation2d flipTranslation(Translation2d translation) {
        return flipTranslation(translation, CURRENT_SYMMETRY);
    }

    public static Rotation2d flipRotation(Rotation2d rotation) {
        return flipRotation(rotation, CURRENT_SYMMETRY);
    }

    public static Pose2d flipPose(Pose2d pose) {
        return flipPose(pose, CURRENT_SYMMETRY);
    }

    public static ChassisSpeeds flipSpeeds(ChassisSpeeds speeds) {
        return flipSpeeds(speeds, CURRENT_SYMMETRY);
    }

    public static ChassisAccels flipAccels(ChassisAccels accels) {
        return flipAccels(accels, CURRENT_SYMMETRY);
    }

    public static Translation2d flipTranslation(Translation2d translation, FieldSymmetry symmetry) {
        return switch (symmetry) {
            case MIRRORED -> new Translation2d(FieldLayout.kFieldLength - translation.getX(), translation.getY());
            case ROTATIONAL -> new Translation2d(FieldLayout.kFieldLength - translation.getX(), FieldLayout.kFieldWidth - translation.getY());
        };
    }

    public static Rotation2d flipRotation(Rotation2d rotation, FieldSymmetry symmetry) {
        return switch (symmetry) {
            case MIRRORED -> new Rotation2d(-rotation.getCos(), rotation.getSin());
            case ROTATIONAL -> rotation.minus(Rotation2d.kPi);
        };
    }

    public static Pose2d flipPose(Pose2d pose, FieldSymmetry symmetry) {
        return new Pose2d(
            flipTranslation(pose.getTranslation(), symmetry),
            flipRotation(pose.getRotation(), symmetry)
        );
    }

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
