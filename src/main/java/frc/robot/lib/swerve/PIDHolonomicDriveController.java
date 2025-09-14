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
import frc.robot.lib.control.PidvController;
import frc.robot.lib.control.ProfiledPIDVController;
import frc.robot.lib.trajectory.RedTrajectory;

public class PIDHolonomicDriveController implements DriveController {
    private final PidvController xController;
    private final PidvController yController;
    private final ProfiledPIDVController thetaController;
    private double accelConstant;

    private RedTrajectory trajectory;
    private Pose2d currentPose;
    private ChassisSpeeds currentSpeeds;

    private Timer timer = null;

    static Field2d field = new Field2d();
    static {
		SmartDashboard.putData("debug", field);
    }

    /**
     * A drive controller that works with 2 {@link PidvController}s for translation and one {@link ProfiledPIDVController} for rotation.
     * @param translationConstants The {@link PIDFConstants} for the translation of the robot.
     * @param rotationConstants The {@link ProfiledPIDFConstants} for the rotation of the robot.
     * @param accelConstant The acceleration feedforwards (useful for traversing sharp turns on a trajectory).
     */
    public PIDHolonomicDriveController(PIDFConstants translationConstants, ProfiledPIDFConstants rotationConstants, double accelConstant) {
        xController = new PidvController(translationConstants);
        yController = new PidvController(translationConstants);
        thetaController = new ProfiledPIDVController(rotationConstants);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.accelConstant = accelConstant;

        timer = new Timer();
    }

    @Override
    public void setTrajectory(RedTrajectory trajectory) {
        this.trajectory = trajectory;
        timer.reset();
        timer.start();
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
        if (trajectory == null || currentPose == null || currentSpeeds == null || trajectory.isDone()) {
            return new ChassisSpeeds();
        }

        RedTrajectory.State targetState = trajectory.advanceTo(timer.get());

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

        xController.setTarget(targetState.pose.getX());
        yController.setTarget(targetState.pose.getY());

        xController.setFeedforward(vxFF);
        yController.setFeedforward(vyFF);

        xController.setInput(new Pair<Double, Double>(currentPose.getX(), currentSpeeds.vxMetersPerSecond));
        yController.setInput(new Pair<Double, Double>(currentPose.getY(), currentSpeeds.vyMetersPerSecond));

        double vx = xController.getOutput();
        double vy = yController.getOutput();

        thetaController.setTarget(targetState.pose.getRotation().getRadians());
        thetaController.setFeedforward(targetState.speeds.omegaRadiansPerSecond);
        thetaController.setInput(
            new Pair<Double, Double>(
                currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond));

        double rotation = thetaController.getOutput();

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
        if (trajectory == null) return true;
        if (trajectory.isDone()) {
            System.out.println("Done with trajectory, error: " + Math.hypot(xController.error, yController.error));
            return true;
        }
        return false;
    }

    @Override
    public void reset() {
        xController.reset();
        yController.reset();
        thetaController.reset();
    
        trajectory = null;
        currentPose = null;
        currentSpeeds = null;
    
        if (timer != null) {
            timer.stop();
            timer.reset();
        }
    }

    @Override
    public RedTrajectory getTrajectory() {
        return trajectory;
    }
}
