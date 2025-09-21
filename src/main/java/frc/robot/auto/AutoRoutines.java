package frc.robot.auto;

import frc.robot.lib.trajectory.RedTrajectory;
import frc.robot.lib.trajectory.TrajectoryLoader;
import frc.robot.lib.trajectory.RedTrajectory.TrajectoryType;
import frc.robot.subsystems.drive.commands.AutopilotCommand;
import frc.robot.subsystems.drive.commands.TrajectoryCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class AutoRoutines {
	/** An auto that runs a single test trajectory. */
	public static Command driveAuto() {
		RedTrajectory traj = TrajectoryLoader.loadAutoTrajectory(TrajectoryType.PATHPLANNER, 
			"zisen").get();
		return new TrajectoryCommand(traj);
	}

	public static Command autopilotAuto() {
		// return Commands.none();
		return new AutopilotCommand(new Pose2d(1, 1, Rotation2d.fromDegrees(90)));
	}
}
