package frc.robot.subsystems.drive.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.lib.control.ControlConstants.ProfiledPIDFConstants;
import frc.robot.lib.control.ProfiledPIDVController;
import frc.robot.subsystems.drive.Drive;

public class PIDToPoseCommand extends Command {
    public final Drive drive;

    private final SwerveRequest.ApplyFieldSpeeds request = 
        new SwerveRequest.ApplyFieldSpeeds();

    private final ProfiledPIDVController xController;
    private final ProfiledPIDVController yController;
    private final ProfiledPIDVController thetaController;

    private final Pose2d target;
    private Pose2d currentPose;
    private ChassisSpeeds currentSpeeds;

    public PIDToPoseCommand(Pose2d target) {
        this(
            Drive.getInstance(), 
            target,
            Constants.Auto.PROFILED_TRANSLATION_CONSTANTS, 
            Constants.Auto.ROTATION_CONSTANTS);
    }
    
    public PIDToPoseCommand(Drive drive, Pose2d target, ProfiledPIDFConstants translationConstants, ProfiledPIDFConstants rotationConstants) {
        this.drive = drive;
        this.target = target;
        xController = new ProfiledPIDVController(translationConstants);
        yController = new ProfiledPIDVController(translationConstants);
        thetaController = new ProfiledPIDVController(rotationConstants);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
        setName("PID to " + target.toString());
    }

    @Override
    public void initialize() {
        drive.setSwerveRequest(request);
    }

    @Override
    public void execute() {
        setRobotState(
            drive.getPose(), drive.getFieldSpeeds());
        // updates the request
        request.withSpeeds(calculateSpeeds());
    }

    public void setRobotState(Pose2d pose, ChassisSpeeds speeds) {
        this.currentPose = pose;
        this.currentSpeeds = speeds;
    }

    public ChassisSpeeds calculateSpeeds() {
        if (target == null || currentPose == null || currentSpeeds == null) {
            return new ChassisSpeeds();
        }
        
        xController.setTarget(target.getX());
        yController.setTarget(target.getY());

        xController.setInput(new Pair<Double, Double>(currentPose.getX(), currentSpeeds.vxMetersPerSecond));
        yController.setInput(new Pair<Double, Double>(currentPose.getY(), currentSpeeds.vyMetersPerSecond));

        double vx = xController.getOutput();
        double vy = yController.getOutput();

        thetaController.setTarget(target.getRotation().getRadians());
        thetaController.setInput(
            new Pair<Double, Double>(
                currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond));

        double rotation = thetaController.getOutput();

        return ChassisSpeeds.fromFieldRelativeSpeeds(
            vx,
            vy,
            rotation,
            currentPose.getRotation());
    }

    @Override
    public boolean isFinished() {
        if (Math.hypot(xController.error, yController.error) < 0.03 
            && MathUtil.isNear(thetaController.error / Math.PI * 180, 0, 3.0)) 
        {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.setSwerveRequest(new SwerveRequest.FieldCentric());
    }

    public Pose2d getTarget() {
        return target;
    }
}
