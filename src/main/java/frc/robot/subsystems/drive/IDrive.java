package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.trajectory.RedTrajectory;

public abstract class IDrive extends SubsystemBase {
    /**
     * Sets the target chassis speeds of the robot.
     * @param chassisSpeeds The desired chassis speeds, robot-relative.
     */
    protected abstract void setTargetSpeeds(ChassisSpeeds chassisSpeeds);    

    /**
    * Sets the target states of the individual modules.
    * @param chassisSpeeds The filtered chassis speeds, robot-relative.
    */
    protected abstract void setModuleTargetStates(ChassisSpeeds chassisSpeeds);

    /**
     * Sets the robot to {@code PATH_FOLLOWING} mode, and follows the trajectory.
     * @param trajectory The trajectory to follow.
     */
    protected abstract void setTrajectory(RedTrajectory trajectory);

    /**
     * Sets the robot to {@code TELEOP} mode. It will now run the default command.
     */
    protected abstract void setTeleop();

    /**
     * A command that updates the target speeds, based on input from the controller.
     * @param x The x component of the target chassis speeds, field-relative.
     * @param y The y component of the target chassis speeds, field-relative.
     * @param theta The rotation component of the target chassis speeds, field-relative.
     */
    public abstract Command teleopCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta);

    /**
     * A command that follows a trajectory.
     * @param trajectory The trajectory to follow.
     */
    public abstract Command trajectoryCommand(RedTrajectory trajectory);

    /**
     * Something that links the WheelTracker and the RobotState??? idk what this does
     * @param pose The current pose from RobotState.
     */
    public abstract void setOdometry(Pose2d pose);
}
