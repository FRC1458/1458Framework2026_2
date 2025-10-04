package frc.robot;

import static frc.robot.Robot.controller;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.drive.Drive;
//import frc.robot.subsystems.drive.commands.TeleopCommand;
import frc.robot.subsystems.drive.ctre.CtreDrive.SysIdRoutineType;

public class ControlsMapping {
	public static void mapTeleopCommand() {
		Drive.getInstance().setDefaultCommand((Drive.getInstance().updateTeleopRequest()));
		// run sysID functions
		Drive.getInstance().getCtreDrive().setSysIdRoutine(SysIdRoutineType.STEER);
		controller.a().onTrue(
			Drive.getInstance().getCtreDrive().sysIdDynamic(Direction.kForward));
		controller.b().onTrue(
			Drive.getInstance().getCtreDrive().sysIdDynamic(Direction.kReverse));
		controller.x().onTrue(
			Drive.getInstance().getCtreDrive().sysIdQuasistatic(Direction.kForward));
		controller.y().onTrue(
			Drive.getInstance().getCtreDrive().sysIdQuasistatic(Direction.kReverse));
	}
}
