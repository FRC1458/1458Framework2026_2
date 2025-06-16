package frc.robot.commands;

import java.util.PriorityQueue;
import java.util.Arrays;
import java.util.Collections;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.trajectory.RedTrajectory;
import frc.robot.subsystems.drive.Drive;

public class ExtendedTrajectoryCommand extends Command {
    private final Drive mDrive;
	private final RedTrajectory mTrajectory;
	private final PriorityQueue<TriggerPoint> mTriggers;

	private TriggerPoint currentTriggerPoint;

	public static class TriggerPoint implements Comparable<TriggerPoint> {
		public double timestamp;
		public Command command;
		public TriggerPoint(double timestamp, Command command) {
			this.timestamp = timestamp;
			this.command = command;
		}

		@Override
		public int compareTo(TriggerPoint o) {
			return Double.compare(timestamp, o.timestamp);
		}
	}

	public ExtendedTrajectoryCommand(Drive drive, RedTrajectory trajectory, TriggerPoint... triggers) {
		mDrive = drive;
		mTrajectory = trajectory;
		mTriggers = new PriorityQueue<>(Arrays.asList(triggers));
		currentTriggerPoint = mTriggers.poll();
		
		addRequirements(drive);
		for (TriggerPoint triggerPoint : mTriggers) {
			addRequirements(triggerPoint.command.getRequirements());
		}
	}

	@Override
	public void initialize() {
		mDrive.trajectoryCommand(mTrajectory).schedule();
	}

	@Override
	public void execute() {
		if (currentTriggerPoint != null) {
			while (mTrajectory.progress >= currentTriggerPoint.timestamp) {
				currentTriggerPoint.command.schedule();
				currentTriggerPoint = mTriggers.poll();
			}
		}
	}

	@Override
	public void end(boolean interrupted) {
		if (interrupted) {
			for (TriggerPoint trigger : mTriggers) {
				trigger.command.cancel();
			}
			if (currentTriggerPoint != null) {
				currentTriggerPoint.command.cancel();
			}
		}
	}

	@Override
	public boolean isFinished() {
		return mTrajectory.isDone();
	}
}
