package frc.robot.subsystems.drive.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.lib.control.ControlConstants.PIDFConstants;
import frc.robot.lib.control.ControlConstants.ProfiledPIDFConstants;
import frc.robot.lib.control.PIDVController;
import frc.robot.lib.control.ProfiledPIDVController;
import frc.robot.lib.trajectory.RedTrajectory;
import frc.robot.subsystems.drive.Drive;

/**
 * Command that follows a trajectory
 */
public class TrajectoryCommand extends Command {
    public final Drive drive;

    private final PIDVController xController;
    private final PIDVController yController;
    private final ProfiledPIDVController thetaController;
    private double accelConstant;

    private final RedTrajectory trajectory;
    private Pose2d currentPose;
    private ChassisSpeeds currentSpeeds;

    private Timer timer = null;

    static Field2d field = new Field2d();
    static {
		SmartDashboard.putData("debug", field);
    }

    public TrajectoryCommand(RedTrajectory trajectory) {
        this(
            Drive.getInstance(), 
            trajectory, 
            Constants.Auto.TRANSLATION_CONSTANTS, 
            Constants.Auto.ROTATION_CONSTANTS, 
            Constants.Auto.ACCELERATION_CONSTANT);
    }
    
    /**
     * A drive controller that works with 2 {@link PIDVController}s for translation and one {@link ProfiledPIDVController} for rotation.
     * @param translationConstants The {@link PIDFConstants} for the translation of the robot.
     * @param rotationConstants The {@link ProfiledPIDFConstants} for the rotation of the robot.
     * @param accelConstant The acceleration feedforwards (useful for traversing sharp turns on a trajectory).
     */
    public TrajectoryCommand(Drive drive, RedTrajectory trajectory, PIDFConstants translationConstants, ProfiledPIDFConstants rotationConstants, double accelConstant) {
        this.drive = drive;
        this.trajectory = trajectory;
        xController = new PIDVController(translationConstants);
        yController = new PIDVController(translationConstants);
        thetaController = new ProfiledPIDVController(rotationConstants);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.accelConstant = accelConstant;

        timer = new Timer();
        addRequirements(drive);
        setName("Trajectory " + trajectory.name);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        setRobotState(
            RobotState.getLatestFieldToVehicle(), RobotState.getSmoothedVelocity());
        drive.setSwerveRequest(new SwerveRequest.ApplyFieldSpeeds().withSpeeds(calculateSpeeds()));
    }

    public void setRobotState(Pose2d pose, Twist2d speeds) {
        this.currentPose = pose;
        this.currentSpeeds = new ChassisSpeeds(speeds.dx, speeds.dy, speeds.dtheta);
    }

    public ChassisSpeeds calculateSpeeds() {
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
    public boolean isFinished() {
        if (trajectory == null) {
            return true;
        }
        if (trajectory.isDone()) {
            System.out.println("Done with trajectory, error: " + Math.hypot(xController.error, yController.error));
            return true;
        }
        return false;
    }

    public RedTrajectory getTrajectory() {
        return trajectory;
    }
}
