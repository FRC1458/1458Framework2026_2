package frc.robot.subsystems.drive.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.trajectory.RedTrajectory;
import frc.robot.subsystems.drive.LegacyDrive;

@Deprecated
public class LegacyExtendedTrajectoryCommand extends LegacyTrajectoryCommand {
	private final List<Pair<Double, Command>> triggers;

	@SafeVarargs
	public LegacyExtendedTrajectoryCommand(LegacyDrive drive, RedTrajectory trajectory, Pair<Double, Command>... triggers) {
		super(trajectory);
		
		addRequirements(drive);
		this.triggers = new ArrayList<>(List.of(triggers));
		for (Pair<Double,Command> trigger : triggers) {
			new Trigger(() -> trajectory.progress > trigger.getFirst()).onTrue(trigger.getSecond());
		}
	}

	@Override
	public void end(boolean interrupted) {
		if (interrupted) {
			for (Pair<Double, Command> trigger : triggers) {
				if (trigger.getSecond().isScheduled() && !trigger.getSecond().isFinished()){
					trigger.getSecond().cancel();
				}
			}
		}
	}

	@Override
	public boolean isFinished() {
		return super.isFinished();
	}
}
