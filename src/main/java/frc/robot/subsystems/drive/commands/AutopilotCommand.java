package frc.robot.subsystems.drive.commands;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.therekrab.autopilot.*;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.lib.control.ProfiledPIDVController;
import frc.robot.lib.control.ControlConstants.ProfiledPIDFConstants;
import frc.robot.subsystems.drive.Drive2;

public class AutopilotCommand extends Command {
    public final Drive2 drive;
    private static final APConstraints kConstraints = new APConstraints()
        .withAcceleration(5.0)
        .withJerk(2.0);
    private static final APProfile kProfile = new APProfile(kConstraints)
        .withErrorXY(Centimeters.of(2))
        .withErrorTheta(Degrees.of(0.5))
        .withBeelineRadius(Centimeters.of(8));
    public static final Autopilot kAutopilot = new Autopilot(kProfile);
    public final APTarget target;
    
    private final ProfiledPIDVController thetaController;

    public AutopilotCommand(Pose2d target) {
        this(Drive2.getInstance(), new APTarget(target), Constants.Auto.ROTATION_CONSTANTS);
    }

    public AutopilotCommand(Drive2 drive, APTarget target, ProfiledPIDFConstants rotationConstants) {
        this.drive = drive;
        this.target = target;
        thetaController = new ProfiledPIDVController(rotationConstants);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drive);
        setName("Autopilot to " + target.getReference().toString());
    }

    public void execute() {
        ChassisSpeeds robotRelativeSpeeds = drive.getState().Speeds;
        Pose2d pose = drive.getState().Pose;
        ChassisSpeeds speeds = drive.getState().Speeds;

        APResult output = kAutopilot.calculate(pose, robotRelativeSpeeds, target);

        /* these speeds are field relative */
        double veloX = output.vx().in(MetersPerSecond);
        double veloY = output.vy().in(MetersPerSecond);
        Rotation2d headingReference = output.targetAngle();

        thetaController.setTarget(headingReference.getRadians());
        thetaController.setInput(
            new Pair<Double, Double>(
                pose.getRotation().getRadians(), speeds.omegaRadiansPerSecond));

        double rotation = thetaController.getOutput();

        drive.setSwerveRequest(new SwerveRequest.FieldCentric()
            .withVelocityX(veloX)
            .withVelocityY(veloY)
            .withRotationalRate(veloY));
    }

    public boolean isFinished() {
        return kAutopilot.atTarget(drive.getState().Pose, target);
    }

    public void end(boolean interrupted) {
        drive.setSwerveRequest(new SwerveRequest.FieldCentric());
    }
}