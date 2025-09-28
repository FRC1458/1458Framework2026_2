package frc.robot.lib.swerve;

import frc.robot.lib.trajectory.RedTrajectory;

/**
 * An interface for a drive controller which traverses a trajectory. 
 * It takes the current speed and position and returns the calculated {@code ChassisSpeeds}.
 */
@Deprecated
public interface DriveController {
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

    /**
     * Gets the current trajectory.
     */
    public RedTrajectory getTrajectory();
}
