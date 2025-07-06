package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.trajectory.RedTrajectory;
import frc.robot.subsystems.drive.Drive;

public class ExtendedTrajectoryCommand extends Command {
    private final Drive drive;
	private final RedTrajectory trajectory;
	private final List<Pair<Double, Command>> triggers;

	@SuppressWarnings("unchecked") 
	public ExtendedTrajectoryCommand(Drive drive, RedTrajectory trajectory, Pair<Double, Command>... triggers) {
		this.drive = drive;
		this.trajectory = trajectory;
		
		addRequirements(drive);
		this.triggers = new ArrayList<>(List.of(triggers));
		for (Pair<Double,Command> trigger : triggers) {
			new Trigger(() -> trajectory.progress > trigger.getFirst()).onTrue(trigger.getSecond());
		}
	}

	@Override
	public void initialize() {
		drive.trajectoryCommand(trajectory).schedule();
	}

	@Override
	public void execute() {}

	@Override
	public void end(boolean interrupted) {
		if (interrupted) {
			for (Pair<Double,Command> trigger : triggers) {
				if (trigger.getSecond().isScheduled() && !trigger.getSecond().isFinished()){
					trigger.getSecond().cancel();
				}
			}
		}
	}

	@Override
	public boolean isFinished() {
		return trajectory.isDone();
	}
}
