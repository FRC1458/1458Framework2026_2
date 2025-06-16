package frc.robot.lib.swerve;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.lib.control.IO;
import frc.robot.lib.trajectory.RedTrajectory;

/**
 * An interface for a drive controller which traverses a trajectory. 
 * It takes the current speed and position and returns the calculated {@code ChassisSpeeds}.
 */
public interface DriveController extends IO<Pair<Pose2d, Twist2d>, ChassisSpeeds> {
    /**
     * Sets the trajectory.
     * @param trajectory The trajectory to traverse.
     */
    public void setTrajectory(RedTrajectory trajectory);

    /**
     * Whether the traversal is done.
     * @return Whether the traversal is done.
     */
    public boolean isDone();

    /**
     * Resets the drive controller.
     */
    public void reset();
}
