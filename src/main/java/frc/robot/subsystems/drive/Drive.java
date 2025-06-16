package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.pathfinding.Pathfinding;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.lib.localization.FieldLayout;
import frc.robot.lib.swerve.*;
import frc.robot.lib.trajectory.RedTrajectory;
import frc.robot.lib.util.Conversions;
import frc.robot.lib.util.Util;
import frc.robot.lib.util.interpolation.InterpolatingPose2d;
import frc.robot.subsystems.Cancoders;
import frc.robot.subsystems.WheelTracker;

public class Drive extends IDrive {
    private static Drive mInstance;
    public static Drive getInstance() {
        if (mInstance == null) {
            mInstance = new Drive();
        }
        return mInstance;
    }

    private State mState = State.TELEOP;
    private static enum State {
        DISABLED,
        TELEOP,
        PATH_FOLLOWING,
        X_LOCK
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

    private Cancoders mCancoders;
    private SwerveModule[] mSwerveModules;
    private WheelTracker mWheelTracker;

    private DriveController mDriveController;
    private Pathfinder mPathFinder;

    public SwerveDriveKinematics mKinematics;

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
        
    private Drive() {
        mCancoders = Cancoders.getInstance();
        mPeriodicIO = new PeriodicIO();
        mSwerveModules = new SwerveModule[] {
            new SwerveModule(
                "FL", Constants.Drive.Modules.FrontLeft.CONSTANTS, mCancoders.getFrontLeft()), 
            new SwerveModule(
                "FR", Constants.Drive.Modules.FrontRight.CONSTANTS, mCancoders.getFrontRight()), 
            new SwerveModule(
                "BL", Constants.Drive.Modules.BackLeft.CONSTANTS, mCancoders.getBackLeft()), 
            new SwerveModule(   
                "BR", Constants.Drive.Modules.BackRight.CONSTANTS, mCancoders.getBackRight())
        };
        mKinematics = new SwerveDriveKinematics(Constants.Drive.MODULE_LOCATIONS);
        mAccelLimiter = new SlewRateLimiter(Constants.Drive.MAX_ACCEL);
        mRotationAccelLimiter = new SlewRateLimiter(Constants.Drive.MAX_ROTATION_ACCEL);
        mWheelTracker = new WheelTracker(mSwerveModules);
        mDriveController = new PIDHolonomicDriveController(
            Constants.Auto.TRANSLATION_CONSTANTS, Constants.Auto.ROTATION_CONSTANTS, 0);
        mPathFinder = new LocalADStar();
    }

    /**
     * Does some calculations
     * idk really what this does.
     */
    private void updateOdometry() {
        SwerveModuleState[] moduleStates = getModuleStates();
		Twist2d twist_vel = Conversions.toTwist2d(mKinematics.toChassisSpeeds(moduleStates));
		Translation2d translation_vel = new Translation2d(twist_vel.dx, twist_vel.dy);
		translation_vel = translation_vel.rotateBy(Pigeon.getInstance().getYaw());
        
        Twist2d predictedTwistVelocity = Conversions.toTwist2d(mPeriodicIO.targetSpeeds);
		mPeriodicIO.predictedVelocity =
            Util.logMap(Util.expMap(
                predictedTwistVelocity).rotateBy(
                    Pigeon.getInstance().getYaw()));
        
		mPeriodicIO.measuredVelocity = new Twist2d(
            translation_vel.getX(),
            translation_vel.getY(),
            twist_vel.dtheta);
        
        RobotState.addOdometryUpdate(
            Timer.getFPGATimestamp(),
            new InterpolatingPose2d(mWheelTracker.getRobotPose()),
            mPeriodicIO.measuredVelocity,
            mPeriodicIO.predictedVelocity);
    }

    @Override
    public void periodic() {        
        updateOdometry();
        switch (mState) {
            case DISABLED:
                setTargetSpeeds(new ChassisSpeeds());
                break;
            case TELEOP:
                break;    
            case PATH_FOLLOWING:
                mDriveController.setInput(
                    new Pair<Pose2d, Twist2d>(
                        RobotState.getLatestFieldToVehicle(), 
                        RobotState.getSmoothedVelocity()
                    )
                );
                setTargetSpeeds(mDriveController.getOutput());
                break;
            default:
                setTargetSpeeds(new ChassisSpeeds());
        }
        setModuleTargetStates(mPeriodicIO.targetSpeeds);
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
        Pigeon.getInstance().setSimAngularVelocity(mPeriodicIO.targetSpeeds.omegaRadiansPerSecond);
    }

    @Override
    public Command teleopCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) { 
        return Commands.runOnce(this::setTeleop, this)
            .andThen(Commands.run(() -> {
                        this.setSpeedsFromController(
                            x.getAsDouble(), 
                            y.getAsDouble(),
                            theta.getAsDouble());
                    }, this));
    }

    @Override
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
    
    @Override
    protected void setTeleop() {
        this.mState = State.TELEOP;
        setTargetSpeeds(new ChassisSpeeds());
    }

    @Override
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

    @Override
    public void setOdometry(Pose2d pose) {
        mWheelTracker.resetPose(pose);
        RobotState.addOdometryUpdate(
            Timer.getFPGATimestamp(), 
            new InterpolatingPose2d(mWheelTracker.getRobotPose()),
            mPeriodicIO.measuredVelocity, mPeriodicIO.predictedVelocity
		);
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
    
    @Override
    protected void setTargetSpeeds(ChassisSpeeds speeds) {
        mPeriodicIO.targetSpeeds = limitSpeeds(speeds);
    }

    @Override
    protected void setModuleTargetStates(ChassisSpeeds speeds) {
        SwerveModuleState[] rawTargetModuleStates = mKinematics.toSwerveModuleStates(mPeriodicIO.targetSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            rawTargetModuleStates,
            Constants.Drive.MAX_SPEED
        );
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
}