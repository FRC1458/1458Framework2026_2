package frc.robot;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Controllers;
import frc.robot.commands.*;
import frc.robot.subsystems.TelemetryManager;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionDeviceManager;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
	private static final CommandScheduler commandScheduler = CommandScheduler.getInstance();;
	private static Command autoCommand;
	public static boolean isAuto;

	public static final CommandXboxController controller =
		new CommandXboxController(Controllers.DRIVER_CONTROLLER_PORT);
		
	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	public Robot() {
		if (Robot.isSimulation()) {
			// TODO: remove this
			
		}

		isAuto = false;
		
		RobotState.reset(Timer.getFPGATimestamp(), new Pose2d());
		RobotState.resetKalman();

		Drive.getInstance();
		VisionDeviceManager.getInstance();
		TelemetryManager.getInstance();
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
		commandScheduler.run();
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
		isAuto = true;
		RobotState.setAlliance(DriverStation.getAlliance());
		autoCommand = AutoRoutines.driveAuto();

		if (autoCommand != null) {
			autoCommand.schedule();
		}
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {
		doNothing();
	}

	/** This function is called when autonomous mode ends. */
	@Override
	public void autonomousExit() {
		isAuto = true;
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when teleop starts running. 
		if (autoCommand != null) {
			autoCommand.cancel();
		}

		Drive.getInstance().setDefaultCommand(Drive.getInstance().teleopCommand());
		controller.a().onTrue(Commands.runOnce(() -> DriverStationSim.setAllianceStationId(AllianceStationID.Blue1)));
		controller.b().onTrue(Drive.getInstance().driveToPoseCommand(
			new Pose2d(5.0, 5.0, Rotation2d.fromDegrees(80))));
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {
		doNothing();
	}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {
		doNothing();
	}

	/** This function is called once when the robot is first started up. */
	@Override
	public void simulationInit() {
    	DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
	}

	/** This function is called periodically whilst in simulation. */
	@Override
	public void simulationPeriodic() {
		doNothing();
	}

	public static void doNothing() {}
}
