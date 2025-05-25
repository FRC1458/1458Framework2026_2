package frc.robot.lib.swerve;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.lib.control.IO;
import frc.robot.lib.trajectory.RedTrajectory;

public interface DriveController extends IO<Pair<Pose2d, Twist2d>, ChassisSpeeds> {
    public void setTrajectory(RedTrajectory trajectory);
    public boolean isDone();
}
