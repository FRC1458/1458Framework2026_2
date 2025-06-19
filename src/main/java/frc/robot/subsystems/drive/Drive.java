package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.lib.localization.FieldLayout;
import frc.robot.lib.swerve.*;
import frc.robot.lib.trajectory.RedTrajectory;
import frc.robot.lib.util.Conversions;
import frc.robot.lib.util.Util;

import frc.robot.Cancoders;
import frc.robot.subsystems.RedSubsystemBase;

public class Drive extends RedSubsystemBase {
    public static Drive mDrive;
    public static Drive getInstance() {
        if (mDrive == null) {
            return new Drive();
        }
        return mDrive;
    }

    private State mState = State.TELEOP;
    private static enum State {
        DISABLED,
        TELEOP,
        PATH_FOLLOWING,
        X_LOCK,
        PREPARING
    }

    private PeriodicIO mPeriodicIO;
    private class PeriodicIO {
        ChassisSpeeds targetSpeeds = new ChassisSpeeds();
        SwerveModuleState[] targetModuleStates = new SwerveModuleState[] {
            new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
        };
        public Twist2d measuredVelocity;
        public Twist2d predictedVelocity;
    }

    private final SwerveModule[] mSwerveModules;

    private WheelTracker mWheelTracker;
    private Pigeon mPigeon;
    private Cancoders mCancoders;
    private DriveController mDriveController;
    private Pathfinder mPathFinder;

    private SwerveDriveKinematics mKinematics;

    private SlewRateLimiter mAccelLimiter;
    private SlewRateLimiter mRotationAccelLimiter;
    
    private final StructArrayPublisher<SwerveModuleState> desiredStatesPublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("SmartDashboard/Drive/States_Desired", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> measuredStatesPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("SmartDashboard/Drive/States_Measured", SwerveModuleState.struct).publish();
    private final StructPublisher<Rotation2d> rotationPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("SmartDashboard/Drive/Rotation", Rotation2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> chassisSpeedsPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("SmartDashboard/Drive/ChassisSpeeds", ChassisSpeeds.struct).publish();

    public Drive() {
        mPeriodicIO = new PeriodicIO();
        mCancoders = Cancoders.getInstance();
        mSwerveModules = new SwerveModule[] {
            new SwerveModule(
                Constants.Drive.Modules.FRONT_LEFT, mCancoders.getFrontLeft()), 
            new SwerveModule(
                Constants.Drive.Modules.FRONT_RIGHT, mCancoders.getFrontRight()), 
            new SwerveModule(
                Constants.Drive.Modules.BACK_LEFT, mCancoders.getBackLeft()), 
            new SwerveModule(   
                Constants.Drive.Modules.BACK_RIGHT, mCancoders.getBackRight())
            // make sure to declare in the correct order
        };
        mKinematics = new SwerveDriveKinematics(Constants.Drive.MODULE_LOCATIONS);
        mAccelLimiter = new SlewRateLimiter(Constants.Drive.MAX_ACCEL);
        mRotationAccelLimiter = new SlewRateLimiter(Constants.Drive.MAX_ROTATION_ACCEL);
        mWheelTracker = new WheelTracker(mSwerveModules);
        mPigeon = Pigeon.getInstance();
        mDriveController = new PIDHolonomicDriveController(
            Constants.Auto.TRANSLATION_CONSTANTS, Constants.Auto.ROTATION_CONSTANTS, Constants.Auto.ACCELERATION_CONSTANT);
        mPathFinder = new LocalADStar();
        SmartDashboard.putData(this);
        for (SwerveModule swerveModule : mSwerveModules) {
            swerveModule.register();
        }
        this.register();
    }


    @Override
    public void periodic() {        
        updateOdometry();
        switch (mState) {
            case TELEOP:
                setModuleTargetStates(mPeriodicIO.targetSpeeds);
                break;    
            case PATH_FOLLOWING:
                mDriveController.setInput(
                    new Pair<Pose2d, Twist2d>(
                        RobotState.getLatestFieldToVehicle(), 
                        RobotState.getSmoothedVelocity()
                    )
                );
                setTargetSpeeds(mDriveController.getOutput());
                setModuleTargetStates(mPeriodicIO.targetSpeeds);
                break;
            case X_LOCK:
                break;
            case PREPARING:
                break;
            default:
                setTargetSpeeds(new ChassisSpeeds());
                setModuleTargetStates(mPeriodicIO.targetSpeeds);
        }

        for (int i = 0; i < mSwerveModules.length; i++) {
            SwerveModule swerveModule = mSwerveModules[i];
            SwerveModuleState state = mPeriodicIO.targetModuleStates[i];
            swerveModule.setTargetState(state);
        }

        FieldLayout.mField.setRobotPose(mWheelTracker.getRobotPose());

		SwerveModuleState[] other = new SwerveModuleState[4];
		for (int i = 0; i < mPeriodicIO.targetModuleStates.length; i++) {
			other[i] = mPeriodicIO.targetModuleStates[i];
			other[i].angle = mPeriodicIO.targetModuleStates[i].angle;
		}

        desiredStatesPublisher.set(other);
		chassisSpeedsPublisher.set(mPeriodicIO.targetSpeeds);
		Rotation2d rotation = mWheelTracker.getRobotPose().getRotation();
		rotationPublisher.set(rotation);
		measuredStatesPublisher.set(getModuleStates());
    }

    @Override
    public void simulationPeriodic() {
        mPigeon.setSimAngularVelocity(mPeriodicIO.targetSpeeds.omegaRadiansPerSecond);
    }

    /**
     * A command that updates the target speeds, based on input from the controller.
     * @param x The x component of the target chassis speeds, field-relative.
     * @param y The y component of the target chassis speeds, field-relative.
     * @param theta The rotation component of the target chassis speeds, field-relative.
     */
    public Command teleopCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) { 
        return Commands.runOnce(this::setTeleop, this)
            .andThen(Commands.run(() -> {
                        this.setSpeedsFromController(
                            x.getAsDouble(), 
                            y.getAsDouble(),
                            theta.getAsDouble());
                    }, this));
    }

    /**
     * A command that follows a trajectory.
     * @param trajectory The trajectory to follow.
     */
    public Command trajectoryCommand(RedTrajectory trajectory) {
        if (trajectory == null) {
            System.out.println("the trajectory was.");
            return Commands.none();
        }

        return Commands.runOnce(() -> setTrajectory(trajectory), this)
                       .andThen(new WaitUntilCommand(mDriveController::isDone))
                       .andThen(new InstantCommand(this::setTeleop));
    }

    /**
     * A command to drive to a pose.
     * @param pose The pose to drive to.
     */
    public Command driveToPoseCommand(Pose2d pose) {
        if (pose == null) {
            System.out.println("the pose was.");
            return Commands.none();
        }
        
        Pose2d startingPose = RobotState.getLatestFieldToVehicle();
        mPathFinder.setStartPosition(startingPose.getTranslation());
        mPathFinder.setGoalPosition(pose.getTranslation());

        try {
            RedTrajectory traj = new RedTrajectory(mPathFinder.getCurrentPath(
                Constants.Pathplanner.GLOBAL_CONSTRAINTS, 
                new GoalEndState(0, pose.getRotation()))
                .getIdealTrajectory(Constants.Pathplanner.config).get(), false);
            return trajectoryCommand(traj);
        } catch (Exception e) {
            DriverStation.reportWarning("Trajectory failed to generate while generating command", false);
            return Commands.none();
        }
    }

    /**
     * A command to set the robot into X-lock, meaning to arrange the wheels into an X-shaped configuration.
     */
    public Command xLockCommand() {
        return Commands.runOnce(this::setXLock);
    }

    /**
     * A command that aligns the swerves prior to some precise movement.
     */
    public Command prepareCommand(ChassisSpeeds speeds, double duration) {
        return Commands.runOnce(() -> setPreparing(speeds)).andThen(Commands.waitSeconds(duration));
    }

    /**
     * The command to run a sysid routine of the specified type and direction.
     * @param moduleId The module to run the routine on.
     * @param type The type of routine (DRIVE or ANGLE).
     * @param direction The direction to run in.
     * @return The command.
     */
    public Command sysIdDynamic(int moduleId, SwerveModule.TuningType type, SysIdRoutine.Direction direction) {
        return mSwerveModules[moduleId].sysIdDynamic(type, direction);
    }

    /**
     * The command to run a sysid routine of the specified type and direction.
     * @param moduleId The module to run the routine on.
     * @param type The type of routine (DRIVE or ANGLE).
     * @param direction The direction to run in.
     * @return The command.
     */
    public Command sysIdQuasistatic(int moduleId, SwerveModule.TuningType type, SysIdRoutine.Direction direction) {
        return mSwerveModules[moduleId].sysIdQuasistatic(type, direction);
    }
    
    /**
     * Sets the robot to {@code TELEOP} mode. It will now run the default command.
     */
    protected void setTeleop() {
        this.mState = State.TELEOP;
        setTargetSpeeds(new ChassisSpeeds());
    }

    /**
     * Sets the robot to {@code PATH_FOLLOWING} mode, and follows the trajectory.
     * @param trajectory The trajectory to follow.
     */
    protected void setTrajectory(RedTrajectory trajectory) {
        this.mState = State.PATH_FOLLOWING;
        mDriveController.reset();
        mDriveController.setTrajectory(trajectory);
    }

    /**
     * Sets the robot into X-lock mode.
     */
    private void setXLock() {
        this.mState = State.X_LOCK;
        mPeriodicIO.targetModuleStates = new SwerveModuleState[] {
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45))
        };
    }

    private void setPreparing(ChassisSpeeds speeds) {
        this.mState = State.PREPARING;
        setModuleTargetStatesPreparing(speeds);
    }

    /**
     * Something that links the WheelTracker and the RobotState??? idk what this does
     * @param pose The current pose from RobotState.
     */
    public void setOdometry(Pose2d pose) {
        mWheelTracker.resetPose(pose);
        RobotState.addOdometryUpdate(
            Timer.getFPGATimestamp(), 
            mWheelTracker.getRobotPose(),
            mPeriodicIO.measuredVelocity, mPeriodicIO.predictedVelocity
		);
	}    

    /**
     * Does some calculations
     * idk really what this does.
     */
    private void updateOdometry() {
        SwerveModuleState[] moduleStates = getModuleStates();
		Twist2d twist_vel = Conversions.toTwist2d(mKinematics.toChassisSpeeds(moduleStates));
		Translation2d translation_vel = new Translation2d(twist_vel.dx, twist_vel.dy);
		translation_vel = translation_vel.rotateBy(mPigeon.getYaw());
        
        Twist2d predictedTwistVelocity = Conversions.toTwist2d(mPeriodicIO.targetSpeeds);
		mPeriodicIO.predictedVelocity =
            Util.logMap(Util.expMap(
                predictedTwistVelocity).rotateBy(
                    mPigeon.getYaw()));
        
		mPeriodicIO.measuredVelocity = new Twist2d(
            translation_vel.getX(),
            translation_vel.getY(),
            twist_vel.dtheta);
        
        RobotState.addOdometryUpdate(
            Timer.getFPGATimestamp(),
            mWheelTracker.getRobotPose(),
            mPeriodicIO.measuredVelocity,
            mPeriodicIO.predictedVelocity);
    }

    public void resetPose(Pose2d pose) {
        mWheelTracker.resetPose(pose);
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
                RobotState.getLatestFieldToVehicle().getRotation()
            )
        );
    }

    /**
     * Sets the target chassis speeds of the robot.
     * @param chassisSpeeds The desired chassis speeds, robot-relative.
     */
    protected void setTargetSpeeds(ChassisSpeeds speeds) {
        mPeriodicIO.targetSpeeds = limitSpeeds(speeds);
    }

    /**
    * Sets the target states of the individual modules.
    * @param chassisSpeeds The filtered chassis speeds, robot-relative.
    */
    protected void setModuleTargetStates(ChassisSpeeds speeds) {
        SwerveModuleState[] rawTargetModuleStates = mKinematics.toSwerveModuleStates(mPeriodicIO.targetSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            rawTargetModuleStates,
            Constants.Drive.MAX_SPEED
        );
        mPeriodicIO.targetModuleStates = rawTargetModuleStates;
    }

    protected void setModuleTargetStatesPreparing(ChassisSpeeds speeds) {
        SwerveModuleState[] rawTargetModuleStates = mKinematics.toSwerveModuleStates(mPeriodicIO.targetSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            rawTargetModuleStates,
            Constants.Drive.MAX_SPEED
        );
        for (SwerveModuleState swerveModuleState : rawTargetModuleStates) {
            swerveModuleState.speedMetersPerSecond = 0.0;
        }
        mPeriodicIO.targetModuleStates = rawTargetModuleStates;
    }

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
        double limitedSpeed = mAccelLimiter.calculate(rawSpeed);
        
        double vx = MathUtil.applyDeadband(
            rawVx / rawSpeed * limitedSpeed, Constants.K_EPSILON);
        double vy = MathUtil.applyDeadband(
            rawVy / rawSpeed * limitedSpeed, Constants.K_EPSILON);

        double omega = MathUtil.clamp(
            mRotationAccelLimiter.calculate(rawOmega),
            -Constants.Drive.MAX_ROTATION_SPEED, Constants.Drive.MAX_ROTATION_SPEED
        );

        return new ChassisSpeeds(vx, vy, omega);
    }

    /**
     * Gets the states of each swerve module.
     * @return An array of the {@code SwerveModuleState}s of each swerve module.
     */
    public SwerveModuleState[] getModuleStates() {
		List<SwerveModuleState> states = new ArrayList<>();
		for (SwerveModule mod : mSwerveModules) {
			states.add(mod.getState());
		}
		return states.toArray(new SwerveModuleState[]{});
	}

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Subsystem");
        builder.addStringProperty("/State", () -> mState.name(), null);
        builder.addStringProperty("/Trajectory", () -> mState == State.PATH_FOLLOWING ? mDriveController.getTrajectory().name : "None", null);
    }
}