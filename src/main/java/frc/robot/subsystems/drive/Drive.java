package frc.robot.subsystems.drive;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.lib.localization.CustomADStar;
import frc.robot.lib.localization.FieldLayout;
import frc.robot.lib.swerve.*;
import frc.robot.lib.trajectory.RedTrajectory;
import frc.robot.lib.util.Conversions;
import frc.robot.lib.util.Util;
import frc.robot.subsystems.CancoderManager;
import frc.robot.subsystems.TelemetryManager;
import frc.robot.subsystems.drive.commands.TrajectoryCommand;
public class Drive extends SubsystemBase {
    private static Drive driveInstance;
    public static Drive getInstance() {
        if (driveInstance == null) {
            driveInstance = new Drive();
        }
        return driveInstance;
    }

    // private State state = State.TELEOP;
    // private static enum State {
    //     DISABLED,
    //     TELEOP,
    //     PATH_FOLLOWING,
    //     X_LOCK,
    //     PREPARING
    // }

    private final DriveIO io;
    public static class DriveIO {
        public ChassisSpeeds targetSpeeds = new ChassisSpeeds();
        public SwerveModuleState[] targetModuleStates = new SwerveModuleState[] {
            new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
        public Twist2d measuredVelocity;
        public Twist2d predictedVelocity;
    }

    private final SwerveModule[] swerveModules;

    private final WheelTracker wheelTracker;
    private final Pigeon pigeon;
    private final CancoderManager cancoders;
    // private final DriveController driveController;
    private final CustomADStar pathFinder;

    private final SwerveDriveKinematics swerveKinematics;

    private final SlewRateLimiter accelLimiter;
    private final SlewRateLimiter rotationAccelLimiter;

    public Drive() {
        super();
        io = new DriveIO();
        cancoders = CancoderManager.getInstance();
        swerveModules = new SwerveModule[] {
            new SwerveModule(
                Constants.Drive.ModuleConstants.FRONT_LEFT, cancoders.getFrontLeft()), 
            new SwerveModule(
                Constants.Drive.ModuleConstants.FRONT_RIGHT, cancoders.getFrontRight()), 
            new SwerveModule(
                Constants.Drive.ModuleConstants.BACK_LEFT, cancoders.getBackLeft()), 
            new SwerveModule(   
                Constants.Drive.ModuleConstants.BACK_RIGHT, cancoders.getBackRight())
            // make sure to declare in the correct order
        };
        swerveKinematics = new SwerveDriveKinematics(Constants.Drive.MODULE_LOCATIONS);
        accelLimiter = new SlewRateLimiter(Constants.Drive.MAX_ACCEL);
        rotationAccelLimiter = new SlewRateLimiter(Constants.Drive.MAX_ROTATION_ACCEL);
        wheelTracker = new WheelTracker(swerveModules);
        pigeon = Pigeon.getInstance();
        // driveController = new ProfiledPIDHolonomicDriveController(
        //     Constants.Auto.TRANSLATION_CONSTANTS2, 
        //     Constants.Auto.ROTATION_CONSTANTS, 
        //     Constants.Auto.ACCELERATION_CONSTANT);
        pathFinder = new CustomADStar();

        TelemetryManager.getInstance().addStructPublisher(
            "Drive/Pose", 
            Pose2d.struct, 
            () -> RobotState.getLatestFieldToVehicle());

        TelemetryManager.getInstance().addStructPublisher(
            "Drive/TargetChassisSpeeds", 
            ChassisSpeeds.struct, () -> io.targetSpeeds);
        TelemetryManager.getInstance().addStructPublisher(
            "Drive/ChassisSpeeds", 
            ChassisSpeeds.struct, () -> 
            ChassisSpeeds.fromFieldRelativeSpeeds(
                Util.fromTwist2d(RobotState.getMeasuredVelocity()), RobotState.getLatestFieldToVehicle().getRotation()));
        TelemetryManager.getInstance().addStructPublisher(
            "Drive/Rotation", 
            Rotation2d.struct, () -> wheelTracker.getRobotPose().getRotation());
        TelemetryManager.getInstance().addStructArrayPublisher(
            "Drive/TargetModuleStates", 
            SwerveModuleState.struct, () -> io.targetModuleStates);
        TelemetryManager.getInstance().addStructArrayPublisher(
            "Drive/ModuleStates", 
            SwerveModuleState.struct, this::getModuleStates);

        setDefaultCommand(teleopCommand());
        TelemetryManager.getInstance().addSendable(this);
    }

    @Override
    public void periodic() {        
        updateOdometry();
        // switch (state) {
        //     case TELEOP:
        //         setModuleTargetStates(io.targetSpeeds);
        //         break;    
        //     case PATH_FOLLOWING:
        //         driveController.setInput(
        //             new Pair<Pose2d, Twist2d>(
        //                 RobotState.getLatestFieldToVehicle(), 
        //                 RobotState.getSmoothedVelocity()));
        //         setTargetSpeeds(driveController.getOutput());
        //         setModuleTargetStates(io.targetSpeeds);
        //         break;
        //     case X_LOCK:
        //         break;
        //     case PREPARING:
        //         break;
        //     default:
        //         setTargetSpeeds(new ChassisSpeeds());
        //         setModuleTargetStates(io.targetSpeeds);
        // }
        setModuleTargetStates(io.targetSpeeds);

        for (int i = 0; i < swerveModules.length; i++) {
            SwerveModule swerveModule = swerveModules[i];
            SwerveModuleState state = io.targetModuleStates[i];
            swerveModule.setTargetState(state);
        }

        FieldLayout.field.setRobotPose(wheelTracker.getRobotPose());
    }

    @Override
    public void simulationPeriodic() {
        pigeon.setSimAngularVelocity(io.targetSpeeds.omegaRadiansPerSecond);
    }

    public boolean isInterrupted() {  
        return !edu.wpi.first.wpilibj.RobotState.isAutonomous() && !Util.MathUtils.allCloseTo(
            new Double[] {
                Robot.controller.getLeftX(), 
                Robot.controller.getLeftY(), 
                Robot.controller.getRightX()}, 
                0, Constants.Controllers.DRIVER_DEADBAND);
    }

    /**
     * Sets the target chassis speeds of the robot.
     * @param chassisSpeeds The desired chassis speeds, robot-relative.
     */
    public void setTargetSpeeds(ChassisSpeeds speeds) {
        io.targetSpeeds = limitSpeeds(speeds);
    }

    /**
     * A command that updates the target speeds, based on input from the controller.
     * @param x The x component of the target chassis speeds, field-relative.
     * @param y The y component of the target chassis speeds, field-relative.
     * @param theta The rotation component of the target chassis speeds, field-relative.
     */
    public Command teleopCommand() { 
        return Commands.run(
            () -> {
                setSpeedsFromController(
                    MathUtil.applyDeadband(
                        Robot.controller.getLeftX(), 
                        Constants.Controllers.DRIVER_DEADBAND),
                    MathUtil.applyDeadband(
                        Robot.controller.getLeftY(), 
                        Constants.Controllers.DRIVER_DEADBAND),
                    MathUtil.applyDeadband(
                        Robot.controller.getRightX(), 
                        Constants.Controllers.DRIVER_DEADBAND));
            }, this).withName("Teleop");
    }

    /**
     * A command that follows a trajectory.
     * @param trajectory The trajectory to follow.
     */
    // public Command trajectoryCommand(RedTrajectory trajectory) {
    //     if (trajectory == null) {
    //         DriverStation.reportWarning("Trajectory was null!", false);
    //         return Commands.none();
    //     }

    //     return Commands.runOnce(() -> setTrajectory(trajectory), this)
    //         .andThen(Commands.race(
    //             Commands.waitUntil(driveController::isDone),
    //             Commands.waitUntil(this::isInterrupted)))
    //         .andThen(Commands.runOnce(this::setTeleop));
    // }
    // TODO: to be replaced by a command factory

    /**
     * A command to drive to a pose.
     * @param pose The pose to drive to.
     */
    // public Command driveToPoseCommand(Pose2d pose) {
    //     if (pose == null) {
    //         DriverStation.reportWarning("Pose was null", false);
    //         return Commands.none();
    //     }

    //     return Commands.runOnce(
    //         () -> {     
    //             pathFinder.setIdealStartingState(
    //                 new IdealStartingState(
    //                     Util.twist2dMagnitude(RobotState.getSmoothedVelocity()),
    //                     RobotState.getLatestFieldToVehicle().getRotation()));
    //             pathFinder.setStartPosition(RobotState.getLatestFieldToOdom());
    //             pathFinder.setGoalPosition(pose.getTranslation());})
    //         .andThen(
    //             Commands.race(
    //                 Commands.waitUntil(() -> pathFinder.isNewPathAvailable()),
    //                 Commands.waitSeconds(Constants.Pathplanner.GENERATION_WAIT_TIME)),
    //             Commands.runOnce(() -> {
    //                 try {
    //                     trajectoryCommand(
    //                         new RedTrajectory(
    //                             pathFinder.getCurrentPath(
    //                                 Constants.Pathplanner.GLOBAL_CONSTRAINTS, 
    //                                 new GoalEndState(0, pose.getRotation()))
    //                         .getIdealTrajectory(Constants.Pathplanner.config)
    //                         .get(), 
    //                         false)).schedule();
    //                 } catch (Exception e) {
    //                     DriverStation.reportError("Trajectory failed to generate! "
    //                         + e.getMessage(), true);}}, this));
    // }
    // TODO: fix this

    /**
     * A command to set the robot into X-lock, meaning to arrange the wheels into an X-shaped configuration.
     */
    // public Command xLockCommand() {
    //     return Commands.runOnce(this::setXLock).andThen(Commands.waitUntil(this::isInterrupted));
    // }

    /**
     * A command that aligns the swerves prior to some precise movement.
     */
    // public Command prepareCommand(ChassisSpeeds speeds, double duration) {
    //     return Commands.runOnce(() -> setPreparing(speeds)).andThen(Commands.waitSeconds(duration))
    //         .andThen(Commands.waitUntil(this::isInterrupted));
    // }    
    
    /**
     * The system identification routine for the chassis rotation.
     * @return The routine.
     */
    public SysIdRoutine rotationRoutine() {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                Constants.Calibration.DriveRotation.RAMP_RATE,
                Constants.Calibration.DriveRotation.DYNAMIC_VOLTAGE,
                null,
                state -> SignalLogger.writeString("/Sysid/DriveRotation/State", state.toString())
            ), 
            new SysIdRoutine.Mechanism(
                volts -> io.targetSpeeds = new ChassisSpeeds(
                    0, 0, volts.in(Units.Volts)), 
                    null, this));
    }

    /**
     * The system identification routine for the chassis rotation.
     * @return The routine.
     */
    public SysIdRoutine translationRoutine() {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                Constants.Calibration.DriveTranslation.RAMP_RATE,
                Constants.Calibration.DriveTranslation.DYNAMIC_VOLTAGE,
                null,
                state -> SignalLogger.writeString("/Sysid/DriveTranslation/State", state.toString())), 
            new SysIdRoutine.Mechanism(
                volts -> io.targetSpeeds = new ChassisSpeeds(
                    volts.in(Units.Volts), 0, 0), null, this));
    }

    public Command rotationSysIdDynamic(SysIdRoutine.Direction direction) {
        return rotationRoutine().dynamic(direction);
    }

    /**
     * The command to run a sysid routine of the specified type and direction.
     * @param moduleId The module to run the routine on.
     * @param type The type of routine (DRIVE or ANGLE).
     * @param direction The direction to run in.
     * @return The command.
     */
    public Command moduleSysIdDynamic(int moduleId, SwerveModule.State type, SysIdRoutine.Direction direction) {
        return swerveModules[moduleId].sysIdDynamic(type, direction);
    }

    /**
     * The command to run a sysid routine of the specified type and direction.
     * @param moduleId The module to run the routine on.
     * @param type The type of routine (DRIVE or ANGLE).
     * @param direction The direction to run in.
     * @return The command.
     */
    public Command moduleSysIdQuasistatic(int moduleId, SwerveModule.State type, SysIdRoutine.Direction direction) {
        return swerveModules[moduleId].sysIdQuasistatic(type, direction);
    }
    
    /**
     * Sets the robot to {@code TELEOP} mode. It will now run the default command.
     */
    // private void setTeleop() {
    //     // this.state = State.TELEOP;
    //     setTargetSpeeds(new ChassisSpeeds());
    // }

    /**
     * Sets the robot to {@code PATH_FOLLOWING} mode, and follows the trajectory.
     * @param trajectory The trajectory to follow.
     */
    // private void setTrajectory(RedTrajectory trajectory) {
    //     // this.state = State.PATH_FOLLOWING;
    //     driveController.reset();
    //     driveController.setTrajectory(trajectory);
    // }
    // TODO: remove this

    /**
     * Sets the robot into X-lock mode.
     */
    // private void setXLock() {
    //     // this.state = State.X_LOCK;
    //     io.targetModuleStates = new SwerveModuleState[] {
    //         new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),
    //         new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)),
    //         new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)),
    //         new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45))};
    // }

    // private void setPreparing(ChassisSpeeds speeds) {
    //     // this.state = State.PREPARING;
    //     setModuleTargetStatesPreparing(speeds);
    // }

    /**
     * Something that links the WheelTracker and the RobotState??? idk what this does
     * @param pose The current pose from RobotState.
     */
    public void setOdometry(Pose2d pose) {
        wheelTracker.resetPose(pose);
        RobotState.addOdometryUpdate(
            Timer.getFPGATimestamp(), 
            wheelTracker.getRobotPose(),
            io.measuredVelocity, io.predictedVelocity);
	}    

    /**
     * Does some calculations
     * idk really what this does.
     */
    private void updateOdometry() {
        SwerveModuleState[] moduleStates = getModuleStates();
		Twist2d twist_vel = Conversions.toTwist2d(swerveKinematics.toChassisSpeeds(moduleStates));
		Translation2d translation_vel = new Translation2d(twist_vel.dx, twist_vel.dy);
		translation_vel = translation_vel.rotateBy(pigeon.getYaw());
        
        Twist2d predictedTwistVelocity = Conversions.toTwist2d(io.targetSpeeds);
		io.predictedVelocity =
            Util.logMap(Util.expMap(
                predictedTwistVelocity).rotateBy(
                    pigeon.getYaw()));
		io.measuredVelocity = new Twist2d(
            translation_vel.getX(),
            translation_vel.getY(),
            twist_vel.dtheta);
        
        RobotState.addOdometryUpdate(
            Timer.getFPGATimestamp(),
            wheelTracker.getRobotPose(),
            io.measuredVelocity,
            io.predictedVelocity);
    }

    public void resetPose(Pose2d pose) {
        wheelTracker.resetPose(pose);
    }

    /**
     * Sets target speeds based on controller input.
     * @param x The x component of the target chassis speeds, field-relative.
     * @param y The y component of the target chassis speeds, field-relative.
     * @param theta The rotation component of the target chassis speeds, field-relative.
     */
    public void setSpeedsFromController(double x, double y, double z) {
        setTargetSpeeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                x * Constants.Drive.MAX_SPEED,
                y * Constants.Drive.MAX_SPEED, 
                z * Constants.Drive.MAX_ROTATION_SPEED,
                RobotState.getLatestFieldToVehicle().getRotation()));
    }

    /**
    * Sets the target states of the individual modules.
    * @param chassisSpeeds The filtered chassis speeds, robot-relative.
    */
    private void setModuleTargetStates(ChassisSpeeds speeds) {
        SwerveModuleState[] rawTargetModuleStates = swerveKinematics.toSwerveModuleStates(io.targetSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            rawTargetModuleStates,
            Constants.Drive.MAX_SPEED);
        io.targetModuleStates = rawTargetModuleStates;
    }

    // private void setModuleTargetStatesPreparing(ChassisSpeeds speeds) {
    //     SwerveModuleState[] rawTargetModuleStates = swerveKinematics.toSwerveModuleStates(io.targetSpeeds);
    //     SwerveDriveKinematics.desaturateWheelSpeeds(
    //         rawTargetModuleStates,
    //         Constants.Drive.MAX_SPEED);
    //     for (SwerveModuleState swerveModuleState : rawTargetModuleStates) {
    //         swerveModuleState.speedMetersPerSecond = 0.0;
    //     }
    //     io.targetModuleStates = rawTargetModuleStates;
    // }

    /**
     * Limits the speeds based on kinematic limits.
     * @param rawSpeeds The raw target speeds.
     * @return The limited speeds, abiding by kinematic limits.
     */
    private ChassisSpeeds limitSpeeds(ChassisSpeeds rawSpeeds) {
        double rawVx = rawSpeeds.vxMetersPerSecond;
        double rawVy = rawSpeeds.vyMetersPerSecond;
        double rawOmega = rawSpeeds.omegaRadiansPerSecond;

        double rawSpeed = Math.hypot(rawVx, rawVy);

        double omega = MathUtil.clamp(
            rotationAccelLimiter.calculate(rawOmega),
            -Constants.Drive.MAX_ROTATION_SPEED, Constants.Drive.MAX_ROTATION_SPEED);
        if (rawSpeed < Constants.DEADBAND) {
            return new ChassisSpeeds(0, 0, omega);
        }
        
        double limitedSpeed = accelLimiter.calculate(rawSpeed);
        
        double vx = MathUtil.applyDeadband(
            rawVx / rawSpeed * limitedSpeed, Constants.DEADBAND);
        double vy = MathUtil.applyDeadband(
            rawVy / rawSpeed * limitedSpeed, Constants.DEADBAND);

        return new ChassisSpeeds(vx, vy, omega);
    }

    /**
     * Gets the states of each swerve module.
     * @return An array of the {@code SwerveModuleState}s of each swerve module.
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];
		for (int i = 0; i < swerveModules.length; i++) {
            states[i] = swerveModules[i].getState();
        }
        return states;
	}

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addStringProperty("/Trajectory", () -> {
            Command currentCommand = getCurrentCommand();
            if (currentCommand != null) {
                return currentCommand.getName().contains("Trajectory") ? 
                    ((TrajectoryCommand) currentCommand).getTrajectory().name : "none";
            } else {
                return "none";
            }
        }, null);
    }
}