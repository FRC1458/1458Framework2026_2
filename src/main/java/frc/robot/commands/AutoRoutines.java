package frc.robot.commands;

import frc.robot.subsystems.drive.Drive;
import frc.robot.Constants;
import frc.robot.lib.trajectory.RedTrajectory;
import frc.robot.lib.trajectory.TrajectoryLoader;
import frc.robot.lib.trajectory.RedTrajectory.TrajectoryType;
import edu.wpi.first.wpilibj2.command.Command;

public final class AutoRoutines {
	/** An auto that runs a single test trajectory. */
	public static Command driveAuto() {
		RedTrajectory traj = TrajectoryLoader.loadAutoTrajectory(TrajectoryType.PATHPLANNER, 
			"zisen").get();
		return new TrajectoryCommand(traj);
	}
}
