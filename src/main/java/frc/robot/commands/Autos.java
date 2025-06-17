package frc.robot.commands;

import static frc.robot.subsystems.drive.Drive.mDrive;
import static frc.robot.subsystems.drive.WheelTracker.mWheelTracker;
import frc.robot.Robot;
import frc.robot.lib.trajectory.RedTrajectory;
import frc.robot.lib.trajectory.TrajectoryLoader;
import frc.robot.lib.trajectory.RedTrajectory.TrajectoryType;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
	/** Example static factory for an autonomous command. */
	public static Command exampleAuto(ExampleSubsystem subsystem) {
		return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
	}

	/** An auto that runs a single test trajectory. */
	public static Command driveAuto() {
		try {
			RedTrajectory traj = TrajectoryLoader.loadAutoTrajectory(TrajectoryType.PATHPLANNER, "zisen").get();
			if (Robot.isSimulation()) {
				return mDrive.runOnce(() -> mWheelTracker.resetPose(traj.getInitialState().pose))
							.andThen(mDrive.prepareCommand(traj.getInitialState().speeds, 1))
							.andThen(mDrive.trajectoryCommand(traj));
			}
			return mDrive.prepareCommand(traj.getInitialState().speeds, 1)
						.andThen(mDrive.trajectoryCommand(traj));
		} catch(Exception e) {
			return Commands.none();
		}
	}
}
