package frc.robot.subsystems.drive.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.lib.control.ControlConstants.PIDFConstants;
import frc.robot.lib.control.ControlConstants.ProfiledPIDFConstants;
import frc.robot.lib.control.PIDVController;
import frc.robot.lib.control.ProfiledPIDVController;
import frc.robot.lib.util.Util;
import frc.robot.subsystems.drive.Drive;

/**
 * <p>Default command for the drivetrain in teleop.</p>
 * <p>Features:</p>
 * <ul>
 * <li>Sets speeds from controller</li>
 * <li>Snap to 90 degree angles when moving</li>
 * </ul>
 */
public class TeleopCommand extends Command {
    public final Drive drive;


    private SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric();

    private static final Rotation2d[] SNAP_ANGLES = {
        Rotation2d.fromDegrees(0),
        Rotation2d.fromDegrees(90),
        Rotation2d.fromDegrees(180),
        Rotation2d.fromDegrees(-90)
    };

    public TeleopCommand() {
        this(Drive.getInstance());
    }
    
    /**
     * A drive controller that works with 2 {@link PIDVController}s for translation and one {@link ProfiledPIDVController} for rotation.
     * @param translationConstants The {@link PIDFConstants} for the translation of the robot.
     * @param rotationConstants The {@link ProfiledPIDFConstants} for the rotation of the robot.
     * @param accelConstant The acceleration feedforwards (useful for traversing sharp turns on a trajectory).
     */
    public TeleopCommand(Drive drive) {
        this.drive = drive;
        addRequirements(drive);
        setName("Teleop");
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        var newSpeeds = calculateSpeeds();
        drive.setSwerveRequest(
            request.withVelocityX(newSpeeds.vxMetersPerSecond)
            .withVelocityY(newSpeeds.vyMetersPerSecond)
            .withRotationalRate(newSpeeds.omegaRadiansPerSecond));
    }

    public ChassisSpeeds calculateSpeeds() {
        double rotation = 
            Util.deadBand(
                Robot.controller.getRightX(), 
                Constants.Controllers.DRIVER_DEADBAND) 
            * Constants.Drive.MAX_ROTATION_SPEED;

        // Translation snapping step
        Translation2d translationVector = new Translation2d(
            Robot.controller.getLeftX(),
            Robot.controller.getLeftY());

        if (translationVector.minus(new Translation2d()).getNorm() > Constants.Controllers.DRIVER_DEADBAND) {
            Rotation2d angle = translationVector.getAngle();
            angle = applyAngleSnapping(angle, Rotation2d.fromDegrees(10));

            double magnitude = translationVector.getNorm();
            translationVector = new Translation2d(magnitude, angle);
        } else {
            translationVector = new Translation2d();
        }
        return new ChassisSpeeds(
            translationVector.getY() * Constants.Drive.MAX_SPEED,
            translationVector.getX() * Constants.Drive.MAX_SPEED,
            rotation);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.setSwerveRequest(new SwerveRequest.FieldCentric());
    }

    private Rotation2d applyAngleSnapping(Rotation2d angle, Rotation2d deadband) {
        for (Rotation2d target : SNAP_ANGLES) {
            double diff = Math.abs(angle.minus(target).getRadians());
            if (diff < deadband.getRadians()) {
                return target;
            }
        }
        return angle;
    }
}
