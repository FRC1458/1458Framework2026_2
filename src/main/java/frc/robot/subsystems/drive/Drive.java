package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.swerve.*;
import frc.robot.lib.trajectory.RedTrajectory;

public class Drive extends SubsystemBase {
    private Drive mInstance;
    public Drive getInstance() {
        if (mInstance == null) {
            mInstance = new Drive();
        }
        return mInstance;
    }

    private State mState = State.DISABLED;
    private static enum State {
        DISABLED,
        TELEOP,
        PATH_FOLLOWING
    }

    private PeriodicIO mPeriodicIO;
    private class PeriodicIO {
        ChassisSpeeds targetSpeeds = new ChassisSpeeds();
        SwerveModuleState[] targetModuleStates = new SwerveModuleState[] {
            new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
        };
    }

    private SwerveModule[] mSwerveModules;

    private DriveController mDriveController;
    public SwerveDriveKinematics mKinematics;

    private SlewRateLimiter mAccelLimiter;
    private SlewRateLimiter mRotationAccelLimiter;
		
    private Drive() {
        mPeriodicIO = new PeriodicIO();
        mSwerveModules = new SwerveModule[] {
            new SwerveModule(
                Constants.Drive.Modules.FrontLeft.CONSTANTS), 
            new SwerveModule(
                Constants.Drive.Modules.FrontRight.CONSTANTS), 
            new SwerveModule(
                Constants.Drive.Modules.BackLeft.CONSTANTS), 
            new SwerveModule(   
                Constants.Drive.Modules.BackRight.CONSTANTS)
        };
        mKinematics = new SwerveDriveKinematics(Constants.Drive.MODULE_LOCATIONS);
        mAccelLimiter = new SlewRateLimiter(Constants.Drive.MAX_ACCEL);
        mRotationAccelLimiter = new SlewRateLimiter(Constants.Drive.MAX_ROTATION_ACCEL);
    }

    @Override
    public void periodic() {
        switch (mState) {
            case DISABLED:
                setTargetSpeeds(new ChassisSpeeds());
                break;
            case TELEOP:
                break;    
            case PATH_FOLLOWING:
                setTargetSpeeds(mDriveController.getOutput());
                break;
            default:
                setTargetSpeeds(new ChassisSpeeds());
        }
        for (int i = 0; i < mSwerveModules.length; i++) {
            SwerveModule swerveModule = mSwerveModules[i];
            SwerveModuleState state = mPeriodicIO.targetModuleStates[i];
            swerveModule.setTargetState(state);
        }
    }

    public void setTargetSpeeds(ChassisSpeeds speeds) {
        mPeriodicIO.targetSpeeds = limitSpeeds(speeds);
    }

    public void setTrajectory(RedTrajectory trajectory) {
        mDriveController = new PIDHolonomicDriveController(
            Constants.Auto.TRANSLATION_CONSTANTS, Constants.Auto.ROTATION_CONSTANTS);
        mDriveController.setTrajectory(trajectory);
    }

    public void setModuleTargetStates(ChassisSpeeds speeds) {
        for (int i = 0; i < mSwerveModules.length; i++) {
            SwerveModuleState[] rawTargetModuleStates = mKinematics.toSwerveModuleStates(mPeriodicIO.targetSpeeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(
                rawTargetModuleStates,
                Constants.Drive.MAX_SPEED
            );
        }
    }

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
}