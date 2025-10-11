package frc.robot.auto;

import frc.robot.lib.trajectory.RedTrajectory;
import frc.robot.lib.trajectory.TrajectoryLoader;
import frc.robot.lib.trajectory.RedTrajectory.TrajectoryType;
import frc.robot.subsystems.drive.commands.PIDToPoseCommand;
import frc.robot.subsystems.drive.commands.TrajectoryCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public final class AutoRoutines {
	public static Command testPidToPose() {
		return new PIDToPoseCommand(new Pose2d(3, 0.5, Rotation2d.fromDegrees(50)));
	}

	public static Command testTrajectoryAuto() {
		RedTrajectory traj = TrajectoryLoader.loadAutoTrajectory(TrajectoryType.PATHPLANNER, 
			"zisen").get();
		return new TrajectoryCommand(traj);
	}
}
