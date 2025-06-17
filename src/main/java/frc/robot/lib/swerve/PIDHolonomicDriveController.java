package frc.robot.lib.swerve;

import frc.robot.lib.control.ControlConstants.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.lib.control.PIDVController;
import frc.robot.lib.control.ProfiledPIDVController;
import frc.robot.lib.trajectory.RedTrajectory;

public class PIDHolonomicDriveController implements DriveController {
    private final PIDVController mXController;
    private final PIDVController mYController;
    private final ProfiledPIDVController mThetaController;
    private double accelConstant;

    private RedTrajectory mTrajectory;
    private Pose2d currentPose;
    private ChassisSpeeds currentSpeeds;

    private Timer mTimer = null;

    static Field2d field = new Field2d();
    static {
		SmartDashboard.putData("debug", field);
    }

    /**
     * A drive controller that works with 2 {@link PIDVController}s for translation and one {@link ProfiledPIDVController} for rotation.
     * @param translationConstants The {@link PIDFConstants} for the translation of the robot.
     * @param rotationConstants The {@link ProfiledPIDFConstants} for the rotation of the robot.
     * @param accelConstant The acceleration feedforwards (useful for traversing sharp turns on a trajectory).
     */
    public PIDHolonomicDriveController(PIDFConstants translationConstants, ProfiledPIDFConstants rotationConstants, double accelConstant) {
        mXController = new PIDVController(translationConstants);
        mYController = new PIDVController(translationConstants);

        mThetaController = new ProfiledPIDVController(
                rotationConstants);
                
        mThetaController.enableContinuousInput(-Math.PI, Math.PI);

        this.accelConstant = accelConstant;

        mTimer = new Timer();
    }

    @Override
    public void setTrajectory(RedTrajectory trajectory) {
        this.mTrajectory = trajectory;
        mTimer.reset();
        mTimer.start();
    }

    public void setRobotState(Pose2d pose, Twist2d speeds) {
        this.currentPose = pose;
        this.currentSpeeds = new ChassisSpeeds(speeds.dx, speeds.dy, speeds.dtheta);
    }

    public void setInput(Pair<Pose2d, Twist2d> current) {
        setRobotState(current.getFirst(), current.getSecond());
    }

    @Override
    public ChassisSpeeds getOutput() {
        if (mTrajectory == null || currentPose == null || currentSpeeds == null || mTrajectory.isDone()) {
            return new ChassisSpeeds();
        }

        RedTrajectory.State targetState = mTrajectory.advanceTo(mTimer.get());

        double vxFF = targetState.speeds.vxMetersPerSecond;
        double vyFF = targetState.speeds.vyMetersPerSecond;

        double xAccelFF = MathUtil.applyDeadband(
                targetState.accels.ax,    
                Constants.Drive.MAX_ACCEL * 0.5);
        double yAccelFF = MathUtil.applyDeadband(
                targetState.accels.ay,
                Constants.Drive.MAX_ACCEL * 0.5);

        double angularAccel = MathUtil.applyDeadband(
                targetState.accels.alpha,
                Constants.Drive.MAX_ROTATION_ACCEL * 0.5);
        xAccelFF += -angularAccel * targetState.pose.getRotation().getSin();
        yAccelFF += angularAccel * targetState.pose.getRotation().getCos();

        mXController.setTarget(targetState.pose.getX());
        mYController.setTarget(targetState.pose.getY());

        mXController.setFeedforward(vxFF);
        mYController.setFeedforward(vyFF);

        mXController.setInput(new Pair<Double, Double>(currentPose.getX(), currentSpeeds.vxMetersPerSecond));
        mYController.setInput(new Pair<Double, Double>(currentPose.getY(), currentSpeeds.vyMetersPerSecond));

        double vx = mXController.getOutput();
        double vy = mYController.getOutput();

        mThetaController.setTarget(targetState.pose.getRotation().getRadians());
        mThetaController.setFeedforward(targetState.speeds.omegaRadiansPerSecond);
        mThetaController.setInput(
            new Pair<Double, Double>(
                currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond));

        double rotation = mThetaController.getOutput();

        field.setRobotPose(targetState.pose);
		SmartDashboard.putData("debug", field);

        return ChassisSpeeds.fromFieldRelativeSpeeds(
                vx + xAccelFF * accelConstant,
                vy + yAccelFF * accelConstant,
                rotation,
                currentPose.getRotation());
    }

    @Override
    public boolean isDone() {
        if (mTrajectory == null) return true;
        if (mTrajectory.isDone()) {
            System.out.println("Done with trajectory, error: " + Math.hypot(mXController.error, mYController.error));
            return true;
        }
        return false;
    }

    @Override
    public void reset() {
        mXController.reset();
        mYController.reset();
        mThetaController.reset();
    
        mTrajectory = null;
        currentPose = null;
        currentSpeeds = null;
    
        if (mTimer != null) {
            mTimer.stop();
            mTimer.reset();
        }
    }

    @Override
    public RedTrajectory getTrajectory() {
        return mTrajectory;
    }
}
