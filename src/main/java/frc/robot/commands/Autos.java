package frc.robot.commands;

import frc.robot.subsystems.drive.Drive;
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
	public static Command driveAuto(Drive drive) {
		try {
			RedTrajectory traj = TrajectoryLoader.loadAutoTrajectory(TrajectoryType.PATHPLANNER, "zisen").get();
			return drive.trajectoryCommand(traj);
		} catch(Exception e) {
			return Commands.none();
		}
	}
}
