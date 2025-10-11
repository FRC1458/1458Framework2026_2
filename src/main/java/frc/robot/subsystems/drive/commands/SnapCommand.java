package frc.robot.subsystems.drive.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.lib.util.Util;
import frc.robot.subsystems.drive.Drive;

/**
 * A re-implementation of the autopilot. Please don't kill me
 */
public class SnapCommand extends Command {
    private final Drive drive;
    private final Pose2d target;
    private final Rotation2d finalEntry;
    private double maxSpeed;
    private double maxAccel;
    private double rotationRadius;
    private SwerveRequest.FieldCentricFacingAngle request = new SwerveRequest.FieldCentricFacingAngle();
    private Transform2d errorTolerance;
    
    public SnapCommand(Pose2d target, Rotation2d finalEntry) {
        this(
            Drive.getInstance(), 
            target, 
            finalEntry, 
            Constants.Drive.MAX_SPEED, 
            Constants.Drive.MAX_ACCEL, 
            1.5,
            0.04,
            2 * Math.PI / 180);
    }

    public SnapCommand(
        Drive drive, 
        Pose2d target, 
        Rotation2d finalEntry, 
        double maxSpeed, 
        double maxAccel, 
        double rotationRadius,
        double errorXY,
        double errorTheta) 
    {
        this.drive = drive;
        this.target = target;
        this.finalEntry = finalEntry;
        this.maxSpeed = maxSpeed;
        this.maxAccel = maxAccel;
        this.rotationRadius = rotationRadius;
        errorTolerance = new Transform2d(errorXY, errorXY, new Rotation2d(errorTheta));
    }

    @Override
    public void execute() {
        calculate(
            drive.getPose(), 
            drive.getState().Speeds, 
            target, 
            finalEntry);
        drive.setSwerveRequest(request);
    }

    @Override
    public boolean isFinished() {
        var delta = drive.getPose().minus(target);
        return Math.abs(delta.getTranslation().getX()) < errorTolerance.getTranslation().getX()
            && Math.abs(delta.getTranslation().getY()) < errorTolerance.getTranslation().getY()
            && Math.abs(delta.getRotation().getRadians()) < errorTolerance.getRotation().getRadians();
    }

    @Override
    public void end(boolean interrupted) {
        drive.setSwerveRequest(new SwerveRequest.FieldCentric());
    }
    
    private void calculate(
        Pose2d pose, 
        ChassisSpeeds speeds,
        Pose2d target, 
        Rotation2d finalEntry) 
    {
        Transform2d delta = target.minus(pose);
        Translation2d offset = delta.getTranslation();
        Rotation2d entryRotation;
        if (!Util.chassisSpeedsEpsilonEquals(speeds, new ChassisSpeeds(), 0.2)) {
            entryRotation = new Rotation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        } else {
            entryRotation = offset.getAngle();
        }
        var deltaRotation = finalEntry.minus(entryRotation);
        var robotRelativeSpeeds = 
            ChassisSpeeds.fromFieldRelativeSpeeds(speeds, pose.getRotation());
        
        double disp;

        double theta = 
            new Rotation2d(offset.getX(), offset.getY())
                .getRadians();
        double radius = offset.getNorm();
        if (theta == 0) {
            disp = radius;
        }
        theta = Math.abs(theta);
        double hypot = Math.hypot(theta, 1);
        double u1 = radius * hypot;
        double u2 = radius * Math.log(theta + hypot) / theta;
        disp = 0.5 * (u1 + u2);
        
        var vx = Math.cos(theta) - theta * Math.sin(theta);
        var vy = theta * Math.cos(theta) + Math.sin(theta);

        var vels = new Translation2d(vx, vy);
        if (vels.getNorm() > Constants.DEADBAND) {
            vels = vels.div(vels.getNorm());
        } else {
            vels = new Translation2d();
        }
        
        double mag;
        double stoppingLimit = Math.sqrt(2.0 * maxAccel * disp);
        double goalV = Math.min(maxSpeed, stoppingLimit);
        double maxDelta = maxAccel * Constants.DT;
        if (goalV > vels.getNorm() + maxDelta) {
            mag = vels.getNorm() + maxDelta;
        } else if (goalV < vels.getNorm() - maxDelta) {
            mag = vels.getNorm() - maxDelta;
        } else {
            mag = goalV;
        }
        vels = vels.times(mag);

        Rotation2d entryAngle = new Rotation2d(delta.getX(), delta.getY());
        vels = vels.rotateBy(entryAngle);

        ChassisSpeeds fieldRelativeSpeeds =
            new ChassisSpeeds(vels.getX(), vels.getY(), 0.0);

        double blend = Math.min(1.0, offset.getNorm() / rotationRadius);
        Rotation2d targetHeading = pose.getRotation().interpolate(finalEntry, 1.0 - blend);

        request.withVelocityX(fieldRelativeSpeeds.vxMetersPerSecond)
            .withVelocityY(fieldRelativeSpeeds.vyMetersPerSecond)
            .withTargetDirection(targetHeading);
    }
}

