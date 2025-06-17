package frc.robot;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Controllers;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.lib.util.interpolation.InterpolatingPose2d;
import frc.robot.subsystems.ExampleSubsystem;
import static frc.robot.subsystems.drive.Drive.mDrive;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
	private final CommandScheduler mCommandScheduler;
	private Command mAutoCommand;

	private final ExampleSubsystem mExampleSubsystem = new ExampleSubsystem();

	private final CommandXboxController mController =
		new CommandXboxController(Controllers.DRIVER_CONTROLLER_PORT);
	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	public Robot() {
		mCommandScheduler = CommandScheduler.getInstance();

		RobotState.reset(Timer.getFPGATimestamp(), new InterpolatingPose2d());
		RobotState.resetKalman();

		var x = mExampleSubsystem;
		var y = mDrive; // loads them in
	}

	/**
	 * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
	 * that you want ran during disabled, autonomous, teleoperated and test.
	 *
	 * <p>This runs after the mode specific periodic functions, but before LiveWindow and
	 * SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		// Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
		// commands, running already-scheduled commands, removing finished or interrupted commands,
		// and running subsystem periodic() methods.  This must be called from the robot's periodic
		// block in order for anything in the Command-based framework to work.
		mCommandScheduler.run();
	}

	/** This function is called once each time the robot enters Disabled mode. */
	@Override
	public void disabledInit() {

	}

	/** This function is called periodically during disabled. */
	@Override
	public void disabledPeriodic() {

	}

	/** This autonomous runs the autonomous command selected. */
	@Override
	public void autonomousInit() {
		RobotState.setAlliance(DriverStation.getAlliance());
		mAutoCommand = Autos.driveAuto();

		if (mAutoCommand != null) {
			mAutoCommand.schedule();
		}
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {

	}

	/** This function is called when autonomous mode ends. */
	@Override
	public void autonomousExit() {

	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when teleop starts running. 
		if (mAutoCommand != null) {
			mAutoCommand.cancel();
		}

		new Trigger(mExampleSubsystem::exampleCondition)
			.onTrue(new ExampleCommand(mExampleSubsystem));

		mController.b().whileTrue(mExampleSubsystem.exampleMethodCommand());
		mDrive.setDefaultCommand(mDrive.teleopCommand(mController::getLeftY, mController::getLeftX, mController::getRightY));
		mController.a().onTrue(Commands.runOnce(() -> DriverStationSim.setAllianceStationId(AllianceStationID.Blue1)));
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {

	}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {

	}

	/** This function is called once when the robot is first started up. */
	@Override
	public void simulationInit() {
    	DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
	}

	/** This function is called periodically whilst in simulation. */
	@Override
	public void simulationPeriodic() {

	}
}
